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
 * Team 32008 -- DECODE 2025-26 -- BLUE CLOSE autonomous.
 *
 * 32008's own subsystems, copied verbatim into teamcode/subsystems:
 * {@link AutoAimSubsystem} (turret + hood + shot solve), {@link Shooter}
 * (flywheels), {@link Intake} (roller + both gates). Only the PATH is this
 * team's own.
 *
 * PATH: the 10-segment Pedro Pathing export, split into 7 chains so the robot can
 * stop and shoot. FOUR volleys, after segments 1, 4, 7 and 10 -- the first is the
 * preload. Segments 7 and 10 are BezierCurves; the other eight are lines. Every
 * control point and heading interpolation is copied from the export exactly.
 *
 *   toShoot1  seg 1        start   -> shoot1    23.4 in   then SHOOT (preload)
 *   pickup1   segs 2 + 3   shoot1  -> pickup1   54.2 in
 *   toShoot2  seg 4        pickup1 -> shoot2    29.6 in   then SHOOT
 *   pickup2   segs 5 + 6   shoot2  -> pickup2   84.6 in
 *   toShoot3  seg 7 CURVE  pickup2 -> shoot3    69.8 in   then SHOOT
 *   pickup3   segs 8 + 9   shoot3  -> pickup3  108.4 in
 *   toShoot4  seg 10 CURVE pickup3 -> shoot4    74.5 in   then SHOOT
 *                                              ------- 444.5 in total
 *
 * SHOT TRIGGER: ARRIVAL, not proximity. The old version started a volley on
 * getting within SHOOT_RADIUS of a canonical point; that is no longer even
 * expressible, because the four shoot poses now sit 0.17 to 5.51 in apart -- being
 * at shoot 1 puts the robot inside any workable radius of shoots 2, 3 and 4 as
 * well. The leg ending IS the arrival signal now, which is also what "these are
 * approximate firing points, fire the moment you get there" actually means.
 *
 * NO HESITATION ONCE THERE. The volley opens the gate and then fires as soon as
 * the turret is LOCKED and the flywheel is AT SPEED -- both measured, not waited
 * out on a timer. Aim and flywheel are commanded every loop for the whole approach,
 * so in practice both are already true on arrival. The old fixed 400 ms pre-fire
 * wait and the 800 ms extra on the preload are gone; only the gate's physical
 * travel is still a timer, because nothing on the robot senses gate position.
 *
 * INTAKE RUNS THE WHOLE TIME, moving or stopped, from start() to the last shot.
 * It is commanded every loop next to the flywheel; the fire step just overrides
 * its power for the feed window.
 *
 * WHICH GOAL: the BLUE goal. Shot 1 from 40.0 in, shots 2-4 from 45.5-45.7 in.
 * NOTE those are ~23 in closer than 32008's tuned CLOSE_FIRE_DISTANCE of 68.5,
 * which is where their flywheel and hood polynomials were actually fitted -- the
 * curves still evaluate, but further from their fit point than the old path was.
 * If the close shots go long, that is the first thing to suspect.
 *
 * MECHANISMS ARE NOT GATED ON THE AIM SOLVE. The flywheel is commanded every loop
 * with a shooterHold() fallback, and the intake never stops. Only the FIRE INSTANT
 * consults the solve, and it has READY_TIMEOUT behind it so a solve that never
 * locks costs one timeout instead of the whole auto. An earlier version made the
 * gate itself wait on aim lock, and one bad solve silently killed the shooter,
 * hood, gate and intake together.
 *
 * FIELD FRAME: the kernel states goals in the PINPOINT frame (pinX = pedroY,
 * pinY = 144 - pedroX). Converted to Pedro below. Blue is (8, 136);
 * (136, 136) is the RED goal.
 */
@Autonomous(name = "32008 Blue Close Auto", group = "32008")
@Configurable
public class BlueCloseAuto extends OpMode {

    // Segment 1's own start point. The export's setStartingPose said (72, 8),
    // which is nowhere near where its own first path begins -- Pedro would snap
    // hard at launch. The path's own number wins.
    private static final Pose START_POSE    = new Pose(24.883, 127.003, Math.toRadians(-37));

    private static final Pose SHOOT_1_POSE  = new Pose(35.185, 106.014, Math.toRadians(130));
    private static final Pose MID_1_POSE    = new Pose(45.211,  83.036, Math.toRadians(180));
    private static final Pose PICKUP_1_POSE = new Pose(16.127,  82.820, Math.toRadians(180));

    private static final Pose SHOOT_2_POSE  = new Pose(38.786, 101.943, Math.toRadians(130));
    private static final Pose MID_2_POSE    = new Pose(49.327,  58.933, Math.toRadians(180));
    private static final Pose PICKUP_2_POSE = new Pose( 9.046,  58.416, Math.toRadians(180));

    private static final Pose SHOOT_3_POSE  = new Pose(38.721, 101.785, Math.toRadians(130));
    private static final Pose MID_3_POSE    = new Pose(47.616,  35.413, Math.toRadians(180));
    private static final Pose PICKUP_3_POSE = new Pose( 6.202,  35.016, Math.toRadians(180));

    private static final Pose SHOOT_4_POSE  = new Pose(38.501, 101.897, Math.toRadians(130));

    // Control points for the two BezierCurves, straight from the export.
    //
    // SEG 7 hooks hard at its own end: over the last 2 in of travel the path tangent
    // swings 108 deg -> 45 deg, and the geometric radius falls to 14 in at t=0.90 and
    // 0.4 in at t=1.00. That is because C2 sits only 3.8 in from the endpoint while
    // C1 is 55 in away. Left exactly as exported -- it is the drawn path -- but see
    // the centripetal note on CENTRIPETAL_WARNING below, and pull C2 back toward the
    // middle if the robot fishtails into shoot 3.
    private static final Pose SEG7_C1 = new Pose(64.430, 55.334);
    private static final Pose SEG7_C2 = new Pose(36.084, 99.105);
    // Seg 10 is gentle by comparison -- 166 in minimum radius, nothing to watch.
    private static final Pose SEG10_C1 = new Pose(24.327, 60.832);

    /**
     * Per-leg drive power, handed to followPath. ALREADY THE CEILING: the drivetrain
     * is built with maxPower(1) and measured xVelocity 81.2 / yVelocity 64.1 in/s
     * (pedroPathing/Constants.java), so there is no headroom above 1.0 to unlock --
     * raising this number does nothing. Lower it if a leg needs to be gentler.
     */
    public static double MAX_POWER = 1.0;

    /**
     * Pedro's two deceleration knobs, applied per chain so nothing here leaks into
     * BlueFarAuto or the teleop follower.
     *
     * BRAKING_STRENGTH multiplies the ZERO POWER ACCELERATION -- which is a MEASURED
     * property of this robot (-27.35 forward, -56.36 lateral). Above 1.0 you are
     * asserting it stops harder than it was measured to stop; the cost is overshoot
     * and localization slip at the end of every leg, and every leg here ends at a
     * shoot point or a pickup. BRAKING_START below 1.0 delays the start of braking.
     *
     * TODO(UNTUNED): both left at the team's current values, so this change alters
     *   nothing until somebody measures. To go faster: raise BRAKING_STRENGTH in
     *   0.1 steps from Panels, watching "Speed (in/s)" and the end-of-leg X/Y against
     *   the target pose. Stop one step BEFORE the first overshoot. That is the only
     *   remaining speed lever -- MAX_POWER is already pinned at the ceiling.
     */
    public static double BRAKING_STRENGTH = 1.0;
    public static double BRAKING_START = 1.0;

    /** BLUE goal, PEDRO frame. Must read 8 / 136 -- 136 / 136 is the RED goal. */
    public static double BLUE_GOAL_X = 144.0 - RobotConstants.BLUE_TARGET_Y;
    public static double BLUE_GOAL_Y = RobotConstants.BLUE_TARGET_X;

    /** Turret mounting trim, degrees, passed straight to AutoAim's yawOffset. */
    public static double YAW_OFFSET = 0.0;

    /**
     * The ONLY remaining pre-fire wait, and it is not a guess at readiness -- it is
     * how long the gate servos physically need to travel, which nothing on this robot
     * senses. Kept at 32008's own FAR value of 400 because that is the number already
     * in this file; time the servos and cut it, it is dead time on all four volleys.
     *
     * What used to sit next to it and is now GONE: a fixed 400 ms "hope the flywheel
     * got there" wait and an 800 ms extra on the preload. Both are replaced by the
     * measured lock+spool check in shotComplete().
     */
    public static long GATE_TRAVEL_MS = 400;
    /** Feed window once firing actually starts. */
    public static long TOTAL_SHOOT_TIME_MS = 550;
    /**
     * Fire anyway after this long waiting on lock+spool. Without it, one solve that
     * never locks would hold a volley open until SHOOT_TIMEOUT and cost the next leg.
     */
    public static double READY_TIMEOUT = 1.2;

    // Safety rails. Not from 32008 -- they keep a bad run from eating the period.
    // 444.5 in of path plus four volleys budgets ~15 s, so these are slack, not caps.
    public static double PATH_TIMEOUT = 7.0;
    /**
     * Hard cap on a collection leg -- a snagged intake gives up rather than eating
     * the period.
     *
     * 4.0 -> 5.0, because the collection legs got substantially longer in this path:
     * pickup1 54.2 in, pickup2 84.6 in, pickup3 108.4 in. At 4.0 the longest one
     * needs a 27.1 in/s average through a heading change; at 5.0 it needs 21.7.
     * This is a BAIL-OUT, not a pacer -- raising it does not slow the auto down, it
     * only stops a leg being abandoned early. The 27 s ABORT_DEADLINE is the real
     * backstop.
     */
    public static double INTAKE_TIMEOUT = 5.0;
    public static double SHOOT_TIMEOUT = 4.0;
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

    private int shotPhase = 0;
    private int shotsFired = 0;
    private boolean shooterLive = false;
    private boolean intakeLive = false;
    private String lastTransition = "none";
    private double distanceToShootPoint = 0.0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        intake.init(hardwareMap);
        // aass = true: AutoAim owns turret "lt" AND hood "panel". Without this
        // both classes grab the turret and fight over its run mode.
        shooter.init(hardwareMap, true);
        // Zero the turret HERE and only here, with it parked forward, mirroring their
        // Robot.autoInit() -> shooter.reset(). Put the flag straight back so TeleOp
        // inherits this zero instead of re-zeroing to wherever auto left the turret.
        AutoAimSubsystem.RESET_TURRET_ENCODER_ON_INIT = true;
        autoAim.init(hardwareMap);
        AutoAimSubsystem.RESET_TURRET_ENCODER_ON_INIT = false;

        intake.gateClose();

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized -- BLUE CLOSE, 4 shots");
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

    /**
     * Every control point and heading interpolation is the export's, verbatim.
     * The braking knobs are applied per chain so they cannot leak into BlueFarAuto
     * or the teleop follower, which share pedroPathing/Constants.
     */
    private void buildPaths() {
        toShoot1 = brake(follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(130)))
                .build();

        pickup1 = brake(follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_1_POSE, MID_1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .addPath(new BezierLine(MID_1_POSE, PICKUP_1_POSE))
                .setTangentHeadingInterpolation())
                .build();

        toShoot2 = brake(follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_1_POSE, SHOOT_2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130)))
                .build();

        pickup2 = brake(follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_2_POSE, MID_2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .addPath(new BezierLine(MID_2_POSE, PICKUP_2_POSE))
                .setTangentHeadingInterpolation())
                .build();

        // Seg 7 -- CUBIC. Sweeps out to x = 43.7 before hooking back in. See SEG7_C1.
        toShoot3 = brake(follower.pathBuilder()
                .addPath(new BezierCurve(PICKUP_2_POSE, SEG7_C1, SEG7_C2, SHOOT_3_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130)))
                .build();

        pickup3 = brake(follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_3_POSE, MID_3_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .addPath(new BezierLine(MID_3_POSE, PICKUP_3_POSE))
                .setTangentHeadingInterpolation())
                .build();

        // Seg 10 -- QUADRATIC, and a gentle one: 166 in minimum radius.
        toShoot4 = brake(follower.pathBuilder()
                .addPath(new BezierCurve(PICKUP_3_POSE, SEG10_C1, SHOOT_4_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130)))
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

        // Aim + shooter + intake run EVERY loop, unconditionally. Nothing below is
        // allowed to depend on the state machine, and the state machine is not
        // allowed to depend on the aim solve.
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

        // ALLIANCE HANDOFF. TeleOp (AASSTEST) never picks a colour -- it aims at
        // whatever these hold, so the auto is what decides. Their BLUE_FAR_18 sets
        // the same pair in loop() and stop(). Without this TeleOp aims at whatever
        // the last run left behind. Stated in the PINPOINT frame, which is the
        // frame TeleOp works in, so no conversion.
        RobotConstants.teleOpTargetX = RobotConstants.BLUE_TARGET_X;
        RobotConstants.teleOpTargetY = RobotConstants.BLUE_TARGET_Y;
    }

    private void updateAim() {
        Pose p = follower.getPose();
        Vector v = follower.getVelocity();
        double headingDeg = Math.toDegrees(p.getHeading());
        // The shooter does not sit over the centre of rotation; their teleop applies
        // this at the call site (V2 tests/AASSTEST.java:83).
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
     * Commands the flywheel every loop. Falls back to their shooterHold() rather
     * than to zero when the solve has no target, so a momentary bad solve cannot
     * spin the shooter down mid-volley.
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

    /**
     * Intake runs continuously, moving or stopped -- there is no "collection leg"
     * any more, the roller is simply always on. Called BEFORE the state machine so
     * the fire step can override the power for its feed window and have that stand
     * for the loop.
     */
    private void driveIntake() {
        if (intakeLive) {
            intake.intakeIn();
        }
    }

    /** Telemetry only now -- nothing gates on it. Distance to this leg's shoot pose. */
    private void trackShootPoint(Pose target) {
        Pose p = follower.getPose();
        distanceToShootPoint = Math.hypot(target.getX() - p.getX(), target.getY() - p.getY());
    }

    private boolean pathDone() {
        return pathDone(PATH_TIMEOUT);
    }

    /** Collection legs pass INTAKE_TIMEOUT; driving legs get the longer PATH_TIMEOUT. */
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

    /** Turret on target AND flywheel at the speed the solve asked for. */
    private boolean readyToFire() {
        return aim.hasTarget && aim.isAimLocked && shooter.shooterReady(aim.targetRpm);
    }

    /**
     * One volley: gate open, fire the INSTANT the turret is locked and the flywheel
     * is at speed, feed, close. The only timer standing between arrival and firing
     * is GATE_TRAVEL_MS, which is physical servo travel, not a guess at readiness.
     *
     * READY_TIMEOUT fires anyway if the solve never locks, so a bad solve costs one
     * volley's worth of hesitation instead of the rest of the auto.
     */
    private boolean shotComplete() {
        if (stateTimer.getElapsedTimeSeconds() > SHOOT_TIMEOUT) {
            intake.gateClose();
            shotPhase = 0;
            shotsFired++;
            lastTransition = "shot: ABANDONED on timeout";
            return true;
        }

        switch (shotPhase) {

            case 0:
                intake.gateOpen();
                intake.intakeEngage();
                shotTimer.resetTimer();
                shotPhase = 1;
                lastTransition = "shot: gate opening";
                return false;

            case 1: {
                // Gate must have physically travelled first -- nothing senses it.
                if (shotTimer.getElapsedTime() < GATE_TRAVEL_MS) {
                    return false;
                }
                boolean ready = readyToFire();
                boolean gaveUp = shotTimer.getElapsedTimeSeconds() > READY_TIMEOUT;
                if (ready || gaveUp) {
                    intake.intakeFire(shooter.calculateIntakePower());
                    shotTimer.resetTimer();
                    shotPhase = 2;
                    lastTransition = ready
                            ? "shot: FIRING (locked + at speed)"
                            : "shot: FIRING (READY_TIMEOUT -- not locked)";
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

            // ARRIVAL is the trigger, every time. No radius, no slow-down gate --
            // the four shoot poses are 0.17 to 5.51 in apart and no radius can tell
            // them apart, so the leg ending is what "we are there" means now.
            case DRIVE_TO_SHOOT_1:
                trackShootPoint(SHOOT_1_POSE);
                if (pathDone()) setState(State.SHOOT_1, "arrived: shoot 1");
                break;

            case SHOOT_1:
                if (shotComplete()) {
                    follow(pickup1);
                    setState(State.DRIVE_PICKUP_1, "shot 1 done");
                }
                break;

            case DRIVE_PICKUP_1:
                if (pathDone(INTAKE_TIMEOUT)) {
                    follow(toShoot2);
                    setState(State.DRIVE_TO_SHOOT_2, "pickup 1 done");
                }
                break;

            case DRIVE_TO_SHOOT_2:
                trackShootPoint(SHOOT_2_POSE);
                if (pathDone()) setState(State.SHOOT_2, "arrived: shoot 2");
                break;

            case SHOOT_2:
                if (shotComplete()) {
                    follow(pickup2);
                    setState(State.DRIVE_PICKUP_2, "shot 2 done");
                }
                break;

            case DRIVE_PICKUP_2:
                if (pathDone(INTAKE_TIMEOUT)) {
                    follow(toShoot3);
                    setState(State.DRIVE_TO_SHOOT_3, "pickup 2 done");
                }
                break;

            case DRIVE_TO_SHOOT_3:
                trackShootPoint(SHOOT_3_POSE);
                if (pathDone()) setState(State.SHOOT_3, "arrived: shoot 3");
                break;

            case SHOOT_3:
                if (shotComplete()) {
                    follow(pickup3);
                    setState(State.DRIVE_PICKUP_3, "shot 3 done");
                }
                break;

            case DRIVE_PICKUP_3:
                if (pathDone(INTAKE_TIMEOUT)) {
                    follow(toShoot4);
                    setState(State.DRIVE_TO_SHOOT_4, "pickup 3 done");
                }
                break;

            case DRIVE_TO_SHOOT_4:
                trackShootPoint(SHOOT_4_POSE);
                if (pathDone()) setState(State.SHOOT_4, "arrived: shoot 4");
                break;

            case SHOOT_4:
                if (shotComplete()) {
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
     * mechanisms and holds wherever the robot is. See the class notes.
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
        panelsTelemetry.debug("Speed (in/s)", follower.getVelocity().getMagnitude());
        panelsTelemetry.debug("Dist to shoot pt", distanceToShootPoint);
        panelsTelemetry.debug("Follower busy", follower.isBusy());
        panelsTelemetry.debug("Robot stuck", follower.isRobotStuck());

        panelsTelemetry.debug("Ready to FIRE", readyToFire());
        panelsTelemetry.debug("Flywheel at speed", shooter.shooterReady(aim.targetRpm));
        panelsTelemetry.debug("Intake live", intakeLive);
        panelsTelemetry.debug("Max power", MAX_POWER);
        panelsTelemetry.debug("Braking strength", BRAKING_STRENGTH);
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
