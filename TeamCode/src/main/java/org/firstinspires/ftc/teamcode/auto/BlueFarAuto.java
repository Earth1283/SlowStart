package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoAimSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Team 32008 -- DECODE 2025-26 -- BLUE FAR autonomous.
 *
 * Same subsystems and the same volley machinery as {@link BlueCloseAuto}: 32008's
 * own {@link AutoAimSubsystem}, {@link Shooter} and {@link Intake}, copied verbatim.
 * Only the PATH and the far-range shot timing are this team's own.
 *
 * PATH: the 10-segment Pedro Pathing export, split into 7 chains so the robot can
 * stop and shoot. FOUR volleys, after segments 1, 4, 7 and 10 -- the first is the
 * preload. Segment 8 is the only BezierCurve; the other nine are lines.
 *
 *   toShoot1  seg 1        start   -> shoot1    16.2 in   then SHOOT (preload)
 *   pickup1   segs 2 + 3   shoot1  -> pickup1   54.2 in
 *   toShoot2  seg 4        pickup1 -> shoot2    51.1 in   then SHOOT
 *   pickup2   segs 5 + 6   shoot2  -> pickup2   75.8 in
 *   toShoot3  seg 7        pickup2 -> shoot3    61.0 in   then SHOOT
 *   pickup3   seg 8 CURVE  shoot3  -> pickup3   74.4 in
 *             + seg 9
 *   toShoot4  seg 10       pickup3 -> shoot4    50.1 in   then SHOOT
 *                                              ------- 382.7 in total
 *
 * FAR RANGE IS THE WHOLE DIFFERENCE. All four shots fire from essentially one spot,
 * (59.1, 23.1) at heading 115, at a solved range of 127.5-127.8 in. That lands within
 * 1.3 in of 32008's own tuned FAR_FIRE_DISTANCE of 126.5 -- these shots sit right where
 * their flywheel and hood polynomials were actually fitted, which is the opposite of
 * BlueCloseAuto's situation and a good reason to trust this auto's shot quality more.
 *
 * SPOOL, not settle, is what gates a far shot. The solve asks for ~1878 rpm here
 * against ~1300 on the close side, so the flywheel needs real time to get there and
 * real time to recover after each feed. The drivers measured 1.0-1.8 s. MIN_SPOOL_MS
 * is a hard FLOOR at their lower bound; the measured shooterReady() check carries it
 * the rest of the way, so a shot that needs the full 1.8 s waits for it and one that
 * is already up to speed does not pay for it.
 *
 * SHOT TRIGGER: ARRIVAL, then MEASURED stillness. Same as BlueCloseAuto -- the leg
 * ending starts the volley, the chassis is actively parked (see holdStill), the
 * rotation estimate is re-seeded, and firing waits on chassis stopped + turret locked
 * + flywheel at speed. Every one of those was learned the hard way on the close auto;
 * none of it is speculative here.
 *
 * INTAKE RUNS THE WHOLE TIME, moving or stopped, from start() to the last shot.
 *
 * NO PARK LEG. The export ends at shoot 4 and does not drive anywhere afterwards, so
 * the previous version's PARK_POSE/PARK_DEADLINE are gone -- there is nothing to park
 * to. ABORT_DEADLINE stops the mechanisms and holds position instead.
 *
 * FIELD FRAME: the kernel states goals in the PINPOINT frame (pinX = pedroY,
 * pinY = 144 - pedroX). Converted to Pedro below. Blue is (8, 136);
 * (136, 136) is the RED goal.
 */
@Autonomous(name = "32008 Blue Far Auto", group = "32008")
@Configurable
public class BlueFarAuto extends OpMode {

    // Segment 1's own start point. The export's setStartingPose said (72, 8), which is
    // not where its own first path begins -- Pedro would snap hard at launch. Same
    // discrepancy the close export had; the path's own number wins.
    private static final Pose START_POSE    = new Pose(55.553,  7.404, Math.toRadians(90));

    private static final Pose SHOOT_1_POSE  = new Pose(59.131, 23.177, Math.toRadians(115));
    private static final Pose MID_1_POSE    = new Pose(45.214, 35.586, Math.toRadians(180));
    private static final Pose PICKUP_1_POSE = new Pose( 9.692, 35.338, Math.toRadians(180));

    private static final Pose SHOOT_2_POSE  = new Pose(59.242, 22.904, Math.toRadians(115));
    private static final Pose MID_2_POSE    = new Pose(46.703, 59.222, Math.toRadians(180));
    private static final Pose PICKUP_2_POSE = new Pose( 9.292, 58.342, Math.toRadians(180));

    private static final Pose SHOOT_3_POSE  = new Pose(59.096, 23.116, Math.toRadians(115));
    // Seg 8 ends here at -90, then seg 9 runs straight down the wall to PICKUP_3.
    private static final Pose MID_3_POSE    = new Pose(11.454, 31.069, Math.toRadians(-90));
    private static final Pose PICKUP_3_POSE = new Pose(11.336,  8.390, Math.toRadians(-90));

    private static final Pose SHOOT_4_POSE  = new Pose(59.203, 23.045, Math.toRadians(115));

    // The one curve on this path. Gentle by the standards of the close auto: 7.9 in
    // minimum radius at t = 0.92, saturating centripetal above 34 in/s, and its leg
    // decelerates into the wall anyway. Nothing like the close auto's seg 2 hairpin.
    private static final Pose SEG8_C1 = new Pose(18.190, 43.669);

    /** Per-leg drive power. maxPower(1) in Constants is already the ceiling. */
    public static double MAX_POWER = 1.0;

    /**
     * Pedro's deceleration knobs, applied per chain so nothing leaks into
     * BlueCloseAuto or the teleop follower. BRAKING_STRENGTH multiplies a MEASURED
     * property of this robot; above 1.0 you assert it stops harder than measured.
     *
     * TODO(UNTUNED): left at the team's current values, so this changes nothing until
     *   somebody measures. Raise BRAKING_STRENGTH in 0.1 steps from Panels, watching
     *   end-of-leg X/Y against the target pose, and stop one step before overshoot.
     */
    public static double BRAKING_STRENGTH = 1.0;
    public static double BRAKING_START = 1.0;

    /** BLUE goal, PEDRO frame. Must read 8 / 136 -- 136 / 136 is the RED goal. */
    public static double BLUE_GOAL_X = 144.0 - RobotConstants.BLUE_TARGET_Y;
    public static double BLUE_GOAL_Y = RobotConstants.BLUE_TARGET_X;

    /** Turret mounting trim, degrees, passed straight to AutoAim's yawOffset. */
    public static double YAW_OFFSET = 0.0;

    /** Gate servo travel. Nothing on this robot senses gate position. */
    public static long GATE_TRAVEL_MS = 400;
    /** Feed window once firing actually starts. */
    public static long TOTAL_SHOOT_TIME_MS = 550;

    /**
     * HARD FLOOR on spool-up before a far shot may fire, milliseconds.
     *
     * The drivers' own measurement: a far shot needs 1.0-1.8 s of flywheel spool or it
     * leaves short. This is set at the LOWER bound of that range on purpose, because it
     * is a floor and not the whole answer -- shooterReady() then holds the shot until
     * the flywheel is actually within VELOCITY_TOR of the ~1878 rpm the solve asked
     * for. A shot needing the full 1.8 s waits for it; one already up to speed fires at
     * 1.0 s instead of paying a worst-case tax on every volley.
     *
     * Why far needs this and close does not: the far solve commands ~1878 rpm against
     * ~1300 close, so both the initial spin-up and the post-feed recovery take longer,
     * and the preload leg here is only 16.2 in -- barely any approach to spool during.
     *
     * If far shots still land short, raise this toward 1800 rather than widening
     * VELOCITY_TOR; a wider tolerance would let an under-speed flywheel report ready.
     */
    public static long MIN_SPOOL_MS = 1000;

    /**
     * Fire anyway after this long waiting on stopped + locked + at-speed. Longer than
     * the close auto's 1.2 s because MIN_SPOOL_MS alone eats 1.0 s of it and the
     * measured spool can legitimately run to 1.8 s.
     */
    public static double READY_TIMEOUT = 2.5;

    /** Extra hold AFTER everything reports ready, per shot. */
    public static double SETTLE_SHOOT_1 = 0.0;
    public static double SETTLE_SHOOT_2 = 0.0;
    public static double SETTLE_SHOOT_3 = 0.0;
    public static double SETTLE_SHOOT_4 = 0.0;

    /**
     * WHAT "STOPPED" MEANS, measured off the follower every loop. Same rationale as
     * BlueCloseAuto: pathDone() fires at t > 0.99 with the robot still moving, and
     * cutting power lets it coast rather than stopping it.
     *
     * TODO(UNTUNED): detection thresholds, not measured robot properties. Pedro's own
     *   stuckVelocity default is 1.0 in/s, the nearest reference. Watch the "Speed at
     *   shoot pt" / "Turn rate" telemetry and pull these down to just above the noise
     *   floor. Too tight is SAFE -- READY_TIMEOUT fires the shot anyway.
     */
    public static double STOPPED_SPEED_MAX = 2.0;
    public static double STOPPED_TURN_MAX = 5.0;

    /**
     * Park the turret once everything reports ready, so it cannot hunt through the
     * shot. Their turret feedforward applies static friction as a bang-bang term and
     * limit-cycles otherwise -- see AutoAimSubsystem.holdTurret.
     */
    public static boolean FREEZE_TURRET_ON_LOCK = true;

    // Safety rails. 382.7 in of path plus four far volleys budgets well inside 30 s.
    public static double PATH_TIMEOUT = 7.0;

    /**
     * Per-leg collection time, seconds. Every pickup pose sits hard against a wall
     * (x = 9.7, 9.3, and 11.3 at y = 8.4), so as on the close auto these legs never
     * report parametric end -- they push until the timeout expires, and the timeout IS
     * the leg duration.
     *
     * TODO(UNTUNED): unlike BlueCloseAuto's, these three were NOT stopwatched by the
     *   drivers. They carry forward the single 3.0 s this file already used. Required
     *   average speeds at 3.0 s are 18.1 / 25.3 / 24.8 in/s for legs of 54.2 / 75.8 /
     *   74.4 in, all inside the measured 64-81 in/s drivetrain. Watch a real run: if a
     *   leg reaches the wall early and then sits there, cut it; if it is still short of
     *   the wall when the timer expires, raise it.
     */
    public static double INTAKE_TIMEOUT_1 = 3.0;
    public static double INTAKE_TIMEOUT_2 = 3.0;
    public static double INTAKE_TIMEOUT_3 = 3.0;

    /** Room for MIN_SPOOL + READY_TIMEOUT + the feed window, with margin. */
    public static double SHOOT_TIMEOUT = 5.0;
    public static double ABORT_DEADLINE = 27.0;

    private enum State {
        DRIVE_TO_SHOOT_1, SHOOT_1,
        DRIVE_PICKUP_1,   DRIVE_TO_SHOOT_2, SHOOT_2,
        DRIVE_PICKUP_2,   DRIVE_TO_SHOOT_3, SHOOT_3,
        DRIVE_PICKUP_3,   DRIVE_TO_SHOOT_4, SHOOT_4,
        DONE
    }

    private Follower follower;
    private TelemetryManager panelsTelemetry;

    // 32008's own subsystems, copied verbatim.
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final AutoAimSubsystem autoAim = new AutoAimSubsystem();

    private AutoAimSubsystem.TurretCommand aim = new AutoAimSubsystem.TurretCommand();

    private PathChain toShoot1, pickup1, toShoot2, pickup2,
                     toShoot3, pickup3, toShoot4;

    private State state = State.DRIVE_TO_SHOOT_1;
    private final Timer stateTimer = new Timer();
    private final Timer opmodeTimer = new Timer();
    private final Timer shotTimer = new Timer();
    /** Runs from the moment everything reports ready, not from arrival. */
    private final Timer settleTimer = new Timer();
    private boolean wasReady = false;
    private boolean turretFrozen = false;

    private int shotPhase = 0;
    private int shotsFired = 0;
    private boolean shooterLive = false;
    private boolean intakeLive = false;
    private String lastTransition = "none";
    private double distanceToShootPoint = 0.0;
    private double aimErrorAtFreeze = 0.0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        intake.init(hardwareMap);
        // aass = true: AutoAim owns turret "lt" AND hood "panel". Without this both
        // classes grab the turret and fight over its run mode.
        shooter.init(hardwareMap, true);
        // Zero the turret HERE and only here, with it parked forward. Put the flag
        // straight back so TeleOp inherits this zero instead of re-zeroing.
        AutoAimSubsystem.RESET_TURRET_ENCODER_ON_INIT = true;
        autoAim.init(hardwareMap);
        AutoAimSubsystem.RESET_TURRET_ENCODER_ON_INIT = false;

        intake.gateClose();

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized -- BLUE FAR, 4 shots");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        follower.update();
        panelsTelemetry.debug("Status", "Ready -- park turret FORWARD, robot on LAUNCH LINE");
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Blue goal X (Pedro)", BLUE_GOAL_X);
        panelsTelemetry.debug("Blue goal Y (Pedro)", BLUE_GOAL_Y);
        panelsTelemetry.debug("Turret ticks", autoAim.getCurrentTick());
        panelsTelemetry.debug("Turret deg", autoAim.getCurrentTurretAngle());
        panelsTelemetry.update(telemetry);
    }

    /** Every control point and heading interpolation is the export's, verbatim. */
    private void buildPaths() {
        toShoot1 = brake(follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115)))
                .build();

        pickup1 = brake(follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_1_POSE, MID_1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .addPath(new BezierLine(MID_1_POSE, PICKUP_1_POSE))
                .setTangentHeadingInterpolation())
                .build();

        toShoot2 = brake(follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_1_POSE, SHOOT_2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115)))
                .build();

        pickup2 = brake(follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_2_POSE, MID_2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .addPath(new BezierLine(MID_2_POSE, PICKUP_2_POSE))
                .setTangentHeadingInterpolation())
                .build();

        toShoot3 = brake(follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_2_POSE, SHOOT_3_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115)))
                .build();

        // Seg 8 -- the only curve. Swings left and down to the wall, ending at -90.
        pickup3 = brake(follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_3_POSE, SEG8_C1, MID_3_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(-90))
                .addPath(new BezierLine(MID_3_POSE, PICKUP_3_POSE))
                .setTangentHeadingInterpolation())
                .build();

        toShoot4 = brake(follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_3_POSE, SHOOT_4_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(115)))
                .build();
    }

    /** Applies the two Panels-live braking knobs to a chain under construction. */
    private com.pedropathing.paths.PathBuilder brake(com.pedropathing.paths.PathBuilder b) {
        return b.setBrakingStrength(BRAKING_STRENGTH).setBrakingStart(BRAKING_START);
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        shooterLive = true;
        intakeLive = true;
        opmodeTimer.resetTimer();
        follow(toShoot1);
        setState(State.DRIVE_TO_SHOOT_1, "start");
    }

    /** Every leg goes through here so MAX_POWER applies uniformly. */
    private void follow(PathChain chain) {
        follower.followPath(chain, MAX_POWER, true);
    }

    @Override
    public void loop() {
        follower.update();

        // Aim + shooter + intake run EVERY loop, unconditionally.
        updateAim();
        driveShooter();
        driveIntake();
        publishPoseForTeleOp();

        if (opmodeTimer.getElapsedTimeSeconds() > ABORT_DEADLINE && state != State.DONE) {
            abort();
        } else {
            runStateMachine();
        }

        report();
    }

    @Override
    public void stop() {
        publishPoseForTeleOp();
        shooter.shooterStop();
        intake.intakeStop();
        intake.gateClose();
        autoAim.stop();
    }

    // Static, so it survives into TeleOp -- which seeds its pose from these and
    // cannot auto-aim without them. Stale if TeleOp runs without this auto first.
    private void publishPoseForTeleOp() {
        Pose p = follower.getPose();
        RobotConstants.autoEndX = p.getX();
        RobotConstants.autoEndY = p.getY();
        RobotConstants.autoEndH = p.getHeading();

        // ALLIANCE HANDOFF. TeleOp never picks a colour -- it aims at whatever these
        // hold. Stated in the PINPOINT frame, which is TeleOp's frame, so no conversion.
        RobotConstants.teleOpTargetX = RobotConstants.BLUE_TARGET_X;
        RobotConstants.teleOpTargetY = RobotConstants.BLUE_TARGET_Y;
    }

    private void updateAim() {
        if (turretFrozen) {
            // Motor off; filters, last solve and hood position left exactly as they
            // were so aiming resumes cleanly next state.
            autoAim.holdTurret();
            return;
        }
        Pose p = follower.getPose();
        Vector v = follower.getVelocity();
        double headingDeg = Math.toDegrees(p.getHeading());
        // The shooter does not sit over the centre of rotation.
        double shooterX = p.getX() + Math.cos(p.getHeading()) * RobotConstants.SHOOTER_DRIVETRAIN_OFFSET;
        double shooterY = p.getY() + Math.sin(p.getHeading()) * RobotConstants.SHOOTER_DRIVETRAIN_OFFSET;

        aim = autoAim.update(
                shooterX, shooterY,
                v.getXComponent(), v.getYComponent(),
                headingDeg,
                // Pedro reports heading rate in RADIANS/sec; AutoAim wants degrees.
                Math.toDegrees(follower.getAngularVelocity()),
                BLUE_GOAL_X, BLUE_GOAL_Y,
                false, 0.0,          // isManualMode, manualDist -- always solve from pose
                false,               // isShootOnTheMove -- this auto stops to shoot
                !follower.isBusy(),  // isBraking
                YAW_OFFSET);
    }

    /**
     * Commands the flywheel every loop. Falls back to shooterHold() rather than to
     * zero when the solve has no target, so a momentary bad solve cannot spin the
     * shooter down -- which at far range would cost far more than it does up close.
     */
    private void driveShooter() {
        if (!shooterLive) {
            return;
        }
        if (aim.hasTarget && aim.targetRpm > 0.0) {
            shooter.setShooterVelocity(aim.targetRpm);
        } else {
            shooter.shooterHold();
        }
    }

    /** Intake runs continuously, moving or stopped. The fire step overrides its power. */
    private void driveIntake() {
        if (intakeLive) {
            intake.intakeIn();
        }
    }

    /** Telemetry only -- nothing gates on it. */
    private void trackShootPoint(Pose target) {
        Pose p = follower.getPose();
        distanceToShootPoint = Math.hypot(target.getX() - p.getX(), target.getY() - p.getY());
    }

    private boolean pathDone() {
        return pathDone(PATH_TIMEOUT);
    }

    private boolean pathDone(double timeout) {
        if (follower.isRobotStuck()) {
            lastTransition = "path ended: ROBOT STUCK";
            return true;
        }
        if (stateTimer.getElapsedTimeSeconds() > timeout) {
            lastTransition = "path ended: TIMEOUT (" + timeout + "s)";
            return true;
        }
        if (!follower.isBusy() || follower.atParametricEnd()) {
            lastTransition = "path ended: complete";
            return true;
        }
        return false;
    }

    /** MEASURED stillness, not assumed. */
    private boolean chassisStopped() {
        return follower.getVelocity().getMagnitude() <= STOPPED_SPEED_MAX
                && Math.abs(Math.toDegrees(follower.getAngularVelocity())) <= STOPPED_TURN_MAX;
    }

    /** Chassis stopped AND turret on target AND flywheel actually at the solved rpm. */
    private boolean readyToFire() {
        return chassisStopped()
                && aim.hasTarget && aim.isAimLocked && shooter.shooterReady(aim.targetRpm);
    }

    /** Parks the drivetrain: BRAKE zero-power plus a zero command, not just power off. */
    private void holdStill() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
    }

    /**
     * One far volley. Differs from the close auto in exactly one place: the phase-1
     * hold is max(GATE_TRAVEL_MS, MIN_SPOOL_MS) rather than gate travel alone, so a
     * far shot cannot fire before the flywheel has had the drivers' measured minimum
     * spool time no matter what shooterReady() claims.
     */
    private boolean shotComplete(double settle) {
        // Re-assert the zero drive command every loop of the volley.
        if (shotPhase != 0) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
        }
        if (stateTimer.getElapsedTimeSeconds() > SHOOT_TIMEOUT) {
            intake.gateClose();
            shotPhase = 0;
            shotsFired++;
            lastTransition = "shot: ABANDONED on timeout";
            return true;
        }

        switch (shotPhase) {

            case 0:
                // Stop the chassis for real -- followPath's holdEnd keeps the path PIDF
                // correcting otherwise, and the robot shoots while still being nudged.
                holdStill();
                // Drop the stale rotation estimate so the turret stops counter-rotating
                // against a spin that has ended. Matters most on toShoot4, which turns
                // 155 deg over 50.1 in -- 2x the rotation rate of any other leg here.
                autoAim.resetHeadingFilter();
                intake.gateOpen();
                intake.intakeEngage();
                shotTimer.resetTimer();
                settleTimer.resetTimer();
                wasReady = false;
                turretFrozen = false;
                shotPhase = 1;
                lastTransition = "shot: chassis held, gate opening, spooling";
                return false;

            case 1: {
                // Gate travel AND the far-shot spool floor, whichever is longer.
                long hold = Math.max(GATE_TRAVEL_MS, MIN_SPOOL_MS);
                if (shotTimer.getElapsedTime() < hold) {
                    return false;
                }
                boolean ready = readyToFire();
                if (ready && !wasReady) {
                    wasReady = true;
                    settleTimer.resetTimer();
                    aimErrorAtFreeze = aim.aimError;
                    if (FREEZE_TURRET_ON_LOCK) turretFrozen = true;
                    lastTransition = "shot: READY (stopped+locked+at speed), settling " + settle + "s";
                } else if (!ready && wasReady) {
                    wasReady = false;
                    lastTransition = "shot: readiness LOST, settle restarts";
                }

                boolean settled = wasReady && settleTimer.getElapsedTimeSeconds() >= settle;
                boolean gaveUp = shotTimer.getElapsedTimeSeconds() > READY_TIMEOUT + settle;
                if (settled || gaveUp) {
                    intake.intakeFire(shooter.calculateIntakePower());
                    shotTimer.resetTimer();
                    shotPhase = 2;
                    lastTransition = settled
                            ? "shot: FIRING (spooled + locked)"
                            : "shot: FIRING (READY_TIMEOUT -- may be under speed)";
                }
                return false;
            }

            case 2:
                if (shotTimer.getElapsedTime() >= TOTAL_SHOOT_TIME_MS) {
                    intake.gateClose();
                    intake.intakeDisengage();
                    shotPhase = 0;
                    shotsFired++;
                    lastTransition = "shot " + shotsFired + " done";
                    return true;
                }
                return false;

            default:
                shotPhase = 0;
                return true;
        }
    }

    private void runStateMachine() {
        switch (state) {

            case DRIVE_TO_SHOOT_1:
                trackShootPoint(SHOOT_1_POSE);
                if (pathDone()) setState(State.SHOOT_1, "arrived: shoot 1");
                break;

            case SHOOT_1:
                if (shotComplete(SETTLE_SHOOT_1)) {
                    follow(pickup1);
                    setState(State.DRIVE_PICKUP_1, "shot 1 done");
                }
                break;

            case DRIVE_PICKUP_1:
                if (pathDone(INTAKE_TIMEOUT_1)) {
                    follow(toShoot2);
                    setState(State.DRIVE_TO_SHOOT_2, "pickup 1 done");
                }
                break;

            case DRIVE_TO_SHOOT_2:
                trackShootPoint(SHOOT_2_POSE);
                if (pathDone()) setState(State.SHOOT_2, "arrived: shoot 2");
                break;

            case SHOOT_2:
                if (shotComplete(SETTLE_SHOOT_2)) {
                    follow(pickup2);
                    setState(State.DRIVE_PICKUP_2, "shot 2 done");
                }
                break;

            case DRIVE_PICKUP_2:
                if (pathDone(INTAKE_TIMEOUT_2)) {
                    follow(toShoot3);
                    setState(State.DRIVE_TO_SHOOT_3, "pickup 2 done");
                }
                break;

            case DRIVE_TO_SHOOT_3:
                trackShootPoint(SHOOT_3_POSE);
                if (pathDone()) setState(State.SHOOT_3, "arrived: shoot 3");
                break;

            case SHOOT_3:
                if (shotComplete(SETTLE_SHOOT_3)) {
                    follow(pickup3);
                    setState(State.DRIVE_PICKUP_3, "shot 3 done");
                }
                break;

            case DRIVE_PICKUP_3:
                if (pathDone(INTAKE_TIMEOUT_3)) {
                    follow(toShoot4);
                    setState(State.DRIVE_TO_SHOOT_4, "pickup 3 done");
                }
                break;

            case DRIVE_TO_SHOOT_4:
                trackShootPoint(SHOOT_4_POSE);
                if (pathDone()) setState(State.SHOOT_4, "arrived: shoot 4");
                break;

            case SHOOT_4:
                if (shotComplete(SETTLE_SHOOT_4)) {
                    intakeLive = false;
                    intake.intakeStop();
                    shooterLive = false;
                    shooter.shooterStop();
                    setState(State.DONE, "shot 4 done -- auto complete");
                }
                break;

            case DONE:
            default:
                break;
        }
    }

    /**
     * The export has no park leg, so there is nowhere to run to -- this stops the
     * mechanisms and holds wherever the robot is.
     */
    private void abort() {
        intake.gateClose();
        intakeLive = false;
        intake.intakeStop();
        shooterLive = false;
        shooter.shooterStop();
        shotPhase = 0;
        setState(State.DONE, "ABORT: deadline reached");
    }

    private void setState(State next, String why) {
        // Leaving a volley always hands the turret back to AutoAim.
        turretFrozen = false;
        state = next;
        lastTransition = why;
        stateTimer.resetTimer();
    }

    private void report() {
        panelsTelemetry.debug("State", state);
        panelsTelemetry.debug("Shots fired", shotsFired);
        panelsTelemetry.debug("Shot phase", shotPhase);
        panelsTelemetry.debug("Last transition", lastTransition);
        panelsTelemetry.debug("State time (s)", stateTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("Auto time (s)", opmodeTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Speed at shoot pt", follower.getVelocity().getMagnitude());
        panelsTelemetry.debug("Turn rate (deg/s)", Math.toDegrees(follower.getAngularVelocity()));
        panelsTelemetry.debug("Dist to shoot pt", distanceToShootPoint);
        panelsTelemetry.debug("Follower busy", follower.isBusy());
        panelsTelemetry.debug("Robot stuck", follower.isRobotStuck());

        panelsTelemetry.debug("Ready to FIRE", readyToFire());
        panelsTelemetry.debug("Chassis STOPPED", chassisStopped());
        panelsTelemetry.debug("Turret FROZEN", turretFrozen);
        panelsTelemetry.debug("Aim error at freeze", aimErrorAtFreeze);
        panelsTelemetry.debug("Flywheel at speed", shooter.shooterReady(aim.targetRpm));
        panelsTelemetry.debug("Intake live", intakeLive);
        panelsTelemetry.debug("Min spool (ms)", MIN_SPOOL_MS);
        panelsTelemetry.debug("Aim has target", aim.hasTarget);
        panelsTelemetry.debug("Aim LOCKED", aim.isAimLocked);
        panelsTelemetry.debug("Aim range (in)", aim.targetDist);
        panelsTelemetry.debug("Aim error (deg)", aim.aimError);
        panelsTelemetry.debug("Aim tolerance (deg)", aim.currentTolerance);
        panelsTelemetry.debug("Turret target (deg)", aim.targetTurretAngle);
        panelsTelemetry.debug("Turret actual (deg)", autoAim.getCurrentTurretAngle());

        panelsTelemetry.debug("HOOD cmd (percent)", aim.targetPitch);
        panelsTelemetry.debug("HOOD servo pos", autoAim.hood.getPosition());
        panelsTelemetry.debug("Shooter target", aim.targetRpm);
        panelsTelemetry.debug("Shooter actual", shooter.getShooterVelocity());
        panelsTelemetry.debug("Intake fire power", shooter.calculateIntakePower());
        panelsTelemetry.debug("Battery (V)", autoAim.getCurrentBatteryVoltage());
        panelsTelemetry.update(telemetry);
    }
}
