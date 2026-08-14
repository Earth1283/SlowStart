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

@Autonomous(name = "32008 Blue Close Auto", group = "32008")
@Configurable
public class BlueCloseAuto extends OpMode {

    private static final Pose START_POSE    = new Pose(24.883, 127.003, Math.toRadians(-37));

    private static final Pose SHOOT_1_POSE  = new Pose(38.339, 111.691, Math.toRadians(140));
    private static final Pose MID_1_POSE    = new Pose(45.211,  83.036, Math.toRadians(180));
    private static final Pose PICKUP_1_POSE = new Pose(12.973,  82.610, Math.toRadians(180));

    private static final Pose SHOOT_2_POSE  = new Pose(37.314, 104.466, Math.toRadians(130));
    private static final Pose MID_2_POSE    = new Pose(49.327,  58.933, Math.toRadians(180));
    private static final Pose PICKUP_2_POSE = new Pose( 9.046,  58.416, Math.toRadians(180));

    private static final Pose SHOOT_3_POSE  = new Pose(37.459, 104.308, Math.toRadians(130));
    private static final Pose MID_3_POSE    = new Pose(52.872,  35.203, Math.toRadians(180));
    private static final Pose PICKUP_3_POSE = new Pose( 6.202,  35.016, Math.toRadians(180));

    private static final Pose SHOOT_4_POSE  = new Pose(44.809, 108.625, Math.toRadians(140));

    private static final Pose SEG2_C1 = new Pose(66.167, 79.602);
    //
    // SEG 7'S CUSP IS BACK, and it is now the WORST curve on the path. Moving SHOOT_3
    // to (36.408, 103.887) put the endpoint only 4.8 in from C2 (36.084, 99.105) while
    // C1 stays 55 in away -- the exact geometry that caused this before. The tail:
    //
    //     t=0.85  radius 264 in     t=0.97  radius  4.2 in
    //     t=0.90  radius  39 in     t=1.00  radius  1.1 in   <- the firing point
    //
    // Centripetal saturates above 12.7 in/s here, lower than seg 2's 16.4. Earlier
    // revisions had already pulled this out to 14.5 in radius / 46 in/s; this export
    // undoes that. Applied exactly as drawn. To fix it in the path editor, drag C2
    // back toward the middle of the curve, away from the shoot-3 endpoint.
    private static final Pose SEG7_C1 = new Pose(64.430, 55.334);
    private static final Pose SEG7_C2 = new Pose(36.084, 99.105);
    private static final Pose SEG10_C1 = new Pose(29.373, 55.786);

    public static double MAX_POWER = 1.0;
    public static double BRAKING_STRENGTH = 1.0;
    public static double BRAKING_START = 1.0;

    public static double BLUE_GOAL_X = 144.0 - RobotConstants.BLUE_TARGET_Y;
    public static double BLUE_GOAL_Y = RobotConstants.BLUE_TARGET_X;

    public static double YAW_OFFSET = 0.0;

    public static long GATE_TRAVEL_MS = 400;
    public static long TOTAL_SHOOT_TIME_MS = 550;
    public static double READY_TIMEOUT = 1.2;

    /**
     * PER-SHOT SETTLE, seconds held AFTER the solve locks and the flywheel reaches
     * speed. NOT measured from arrival -- that is what changed.
     *
     * The arrival-based version was chasing the wrong thing. It went 0.5, then 0.9,
     * and the preload still wobbled, because the wobble was never about how long the
     * robot waited: followPath ran with holdEnd = true, so the chassis stayed under
     * active PIDF correction for the whole "wait". Waiting longer only wobbled longer.
     * The chassis is now genuinely parked first (see parkChassis), AutoAim converges
     * against a stationary base, and this is the extra hold on top of the lock.
     *
     * 0.2 on the preload is the driver's number under the new sequence. Shot 3's 0.3
     * carries over. Shots 2 and 4 fire as soon as they lock.
     *
     * Headroom: worst case a volley is READY_TIMEOUT + settle + the 0.55 s feed
     * window = 1.2 + 0.2 + 0.55 = 1.95 s against SHOOT_TIMEOUT's 4.0 s abandon.
     * About 2.2 s of settle is the ceiling before volleys get cut off instead.
     *
     * The settle clock RESTARTS if lock or spool drops out, so this is never a
     * licence to fire on a stale readiness check.
     */
    /**
     * WHAT "STOPPED" MEANS, measured off the follower every loop.
     *
     * This is the term that was missing, and it is why waiting longer never fixed the
     * preload wobble. pathDone() fires on atParametricEnd(), i.e. t > 0.99 -- the
     * robot is still MOVING when the leg is declared over. The original park then cut
     * the drive command, but cutting power does not stop a robot, it lets it coast. So
     * the volley was starting during the coast-down: AutoAim was tracking a chassis
     * that was still drifting, the turret chased it, and the shot went off into that.
     * parkChassis() plus the re-anchor in shotComplete() phase 1 close both halves of
     * that -- see parkChassis() for why the old park made it worse, not just weaker.
     *
     * Nothing was checking. Now readyToFire() will not return true until the measured
     * translational AND angular speeds are both under these, so the settle clock
     * cannot even start while the robot is still moving.
     *
     * TODO(UNTUNED): these are DETECTION THRESHOLDS, not measured robot properties,
     *   and they are starting points, not tuned numbers. Pedro's own stuckVelocity
     *   default is 1.0 in/s, which is the nearest thing to a reference. Watch the new
     *   "Speed at shoot pt" / "Turn rate at shoot pt" telemetry on a real run and pull
     *   these down to just above the noise floor. Too tight is SAFE, not dangerous:
     *   READY_TIMEOUT fires the shot anyway, so a threshold that can never be met
     *   costs one volley's delay, never a hang.
     */
    /**
     * Park the turret the moment the solve locks, instead of letting it keep
     * correcting through the settle and the shot.
     *
     * Their turret feedforward applies static friction as a bang-bang term, so any
     * error above 0.042 deg flips commanded power ~38x and reverses it -- the turret
     * limit-cycles left and right and never settles. See AutoAimSubsystem.holdTurret.
     * Freezing removes the only thing still moving once the chassis has stopped.
     *
     * Aim keeps solving; only the MOTOR is cut. Flywheel rpm and hood hold their last
     * commanded values, which is correct -- the robot is stationary by then, so the
     * solve is not changing anyway. Watch "Aim error at freeze" to confirm it parks
     * somewhere sensible; set this false to get the old always-correcting behaviour.
     */
    public static boolean FREEZE_TURRET_ON_LOCK = true;

    public static double STOPPED_SPEED_MAX = 2.0;
    public static double STOPPED_TURN_MAX = 5.0;

    public static double SETTLE_SHOOT_1 = 0.5;
    public static double SETTLE_SHOOT_2 = 0.0;
    public static double SETTLE_SHOOT_3 = 0.3;
    public static double SETTLE_SHOOT_4 = 0.3;

    public static double PATH_TIMEOUT = 7.0;
    public static double INTAKE_TIMEOUT_1 = 3.7;
    public static double INTAKE_TIMEOUT_2 = 4.5;
    public static double INTAKE_TIMEOUT_3 = 4.6;
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
    private final Timer settleTimer = new Timer();
    private boolean wasReady = false;
    private boolean turretFrozen = false;
    private boolean anchored = false;

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
        shooter.init(hardwareMap, true);
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

    private void buildPaths() {
        toShoot1 = brake(follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(140)))
                .build();

        // Seg 2 -- QUADRATIC, and a hairpin. See SEG2_C1.
        pickup1 = brake(follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_1_POSE, SEG2_C1, MID_1_POSE))
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
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140)))
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

        RobotConstants.teleOpTargetX = RobotConstants.BLUE_TARGET_X;
        RobotConstants.teleOpTargetY = RobotConstants.BLUE_TARGET_Y;
    }

    private void updateAim() {
        if (turretFrozen) {
            // Motor off, everything else -- filters, last solve, hood position --
            // left exactly as it was so aiming can resume cleanly next state.
            autoAim.holdTurret();
            return;
        }
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

    /** MEASURED stillness, not assumed. See STOPPED_SPEED_MAX. */
    private boolean chassisStopped() {
        return follower.getVelocity().getMagnitude() <= STOPPED_SPEED_MAX
                && Math.abs(Math.toDegrees(follower.getAngularVelocity())) <= STOPPED_TURN_MAX;
    }

    /**
     * Chassis actually stopped AND turret on target AND flywheel at the speed the
     * solve asked for. The chassis term is the one that was missing: everything else
     * was already being measured, while "has it stopped moving" was being assumed.
     */
    private boolean readyToFire() {
        return chassisStopped()
                && aim.hasTarget && aim.isAimLocked && shooter.shooterReady(aim.targetRpm);
    }

    /**
     * Parks the drivetrain on Pedro's own position hold, at the pose the robot is
     * actually standing on right now. Ported from BlueFarAuto after the same wobble
     * was traced there; this file had the identical bug.
     *
     * THE OLD startTeleopDrive + ZERO-VECTOR PARK WAS ITSELF A WOBBLE SOURCE. Read
     * Follower.update() (core 2.1.2): the manualDrive branch runs
     *
     *     drivetrain.runDrive(getCentripetalForceCorrection(),
     *                         getTeleopHeadingVector(),
     *                         getTeleopDriveVector(), ...)
     *
     * with NO translational correction at all -- so a zero teleop vector leaves the
     * chassis with no position feedback whatsoever -- and it still applies centripetal.
     * VectorCalculator.getCentripetalForceCorrection() in teleop mode computes
     *
     *     yPrime       = averageVelocity.y / averageVelocity.x
     *     yDoublePrime = averageAcceleration.y / averageVelocity.x
     *     curvature    = yDoublePrime / (sqrt(1 + yPrime^2))^3
     *
     * At a standstill averageVelocity.x is sensor noise near zero. Exact 0/0 gives NaN
     * and is guarded; a TINY NONZERO x is not, and curvature explodes. That feeds
     *
     *     clamp(centripetalScaling * mass * v_tangential^2 * curvature, +/-maxPower)
     *
     * which saturates to FULL SIDEWAYS POWER, aimed along the stale previous path's
     * tangent, sign flipping as noise flips the acceleration estimate -- every loop.
     * So the note above about BRAKE zero-power was true and still not enough: brake
     * resists coasting, but nothing resists a commanded full-power kick.
     *
     * The holdingPosition branch of the same update() runs
     *
     *     drivetrain.runDrive(getTranslationalCorrection() * holdPointTranslationalScaling,
     *                         getHeadingVector() * holdPointHeadingScaling,
     *                         new Vector(), ...)
     *
     * -- real translational and heading correction, softened by Pedro's own 0.45/0.35
     * hold scalings, and NO centripetal term. It is the mode Pedro itself enters at the
     * end of a followPath(holdEnd = true), so this is the library's intended park.
     *
     * Holds the CURRENT pose, not the shoot pose: least motion possible, and AutoAim
     * solves from live pose every loop so arrival error costs nothing in aim.
     *
     * Call ONCE per volley (plus the one re-anchor below). holdPoint() re-snapshots its
     * target and re-runs breakFollowing() on every call, so calling it each loop would
     * drag the hold target along behind the robot.
     */
    private void parkChassis() {
        follower.holdPoint(follower.getPose());
    }

    /**
     * One volley: gate open, fire the INSTANT the turret is locked and the flywheel
     * is at speed, feed, close. The only timer standing between arrival and firing
     * is GATE_TRAVEL_MS, which is physical servo travel, not a guess at readiness.
     *
     * READY_TIMEOUT fires anyway if the solve never locks, so a bad solve costs one
     * volley's worth of hesitation instead of the rest of the auto.
     */
    private boolean shotComplete(double settle) {
        // No per-loop re-assert any more. parkChassis() installs Pedro's holdingPosition
        // mode and follower.update() maintains it for the rest of the volley; nothing in
        // this method clears it. The old re-assert kept re-sending a zero teleop vector,
        // and that whole park is gone.
        if (stateTimer.getElapsedTimeSeconds() > SHOOT_TIMEOUT) {
            intake.gateClose();
            shotPhase = 0;
            shotsFired++;
            lastTransition = "shot: ABANDONED on timeout";
            return true;
        }

        switch (shotPhase) {

            case 0:
                // STOP THE CHASSIS. Arriving at a shoot point did NOT stop it before:
                // followPath was given holdEnd = true, so the follower kept running its
                // PIDF to hold the endpoint, and the robot was still being actively
                // corrected -- and visibly wobbling -- while the shot went off. Adding
                // settle time never fixed that, because nothing was ever asking the
                // drivetrain to stop. This does.
                //
                // parkChassis() uses Pedro's own holdPoint(), which is the only mode
                // that actually corrects position AND applies no centripetal term.
                // Nothing needs undoing afterwards: followPath() calls breakFollowing()
                // internally, which clears the hold when the next leg starts.
                parkChassis();
                // Drop the stale rotation estimate. The chassis has just stopped; the
                // turret must not keep counter-rotating against the spin it arrived
                // with. Matters most on the preload, whose leg turns 177 deg in 20.4 in
                // -- 5.7x the rotation rate of any other leg on this path.
                autoAim.resetHeadingFilter();
                intake.gateOpen();
                intake.intakeEngage();
                shotTimer.resetTimer();
                settleTimer.resetTimer();
                wasReady = false;
                turretFrozen = false;
                anchored = false;
                shotPhase = 1;
                lastTransition = "shot: chassis held, gate opening";
                return false;

            case 1: {
                if (!anchored && chassisStopped()) {
                    anchored = true;
                    parkChassis();
                    autoAim.resetHeadingFilter();
                    lastTransition = "shot: re-anchored at rest (chassis + heading filter)";
                }

                // Gate must have physically travelled first -- nothing senses it.
                if (shotTimer.getElapsedTime() < GATE_TRAVEL_MS) {
                    return false;
                }

                boolean ready = readyToFire();
                if (ready && !wasReady) {
                    wasReady = true;
                    settleTimer.resetTimer();
                    aimErrorAtFreeze = aim.aimError;
                    if (FREEZE_TURRET_ON_LOCK) turretFrozen = true;
                    lastTransition = "shot: LOCKED, turret parked, settling " + settle + "s";
                } else if (!ready && wasReady) {
                    wasReady = false;
                    turretFrozen = false;
                    lastTransition = "shot: lock LOST, turret released, settle restarts";
                }

                boolean settled = wasReady && settleTimer.getElapsedTimeSeconds() >= settle;
                boolean gaveUp = shotTimer.getElapsedTimeSeconds() > READY_TIMEOUT + settle;
                if (settled || gaveUp) {
                    intake.intakeFire(shooter.calculateIntakePower());
                    shotTimer.resetTimer();
                    shotPhase = 2;
                    lastTransition = settled
                            ? "shot: FIRING (locked + settled)"
                            : "shot: FIRING (READY_TIMEOUT -- never locked)";
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
        panelsTelemetry.debug("Speed (in/s)", follower.getVelocity().getMagnitude());
        panelsTelemetry.debug("Dist to shoot pt", distanceToShootPoint);
        panelsTelemetry.debug("Follower busy", follower.isBusy());
        panelsTelemetry.debug("Robot stuck", follower.isRobotStuck());

        panelsTelemetry.debug("Ready to FIRE", readyToFire());
        panelsTelemetry.debug("Chassis STOPPED", chassisStopped());
        panelsTelemetry.debug("Turret FROZEN", turretFrozen);
        panelsTelemetry.debug("Aim error at freeze", aimErrorAtFreeze);
        panelsTelemetry.debug("Speed at shoot pt", follower.getVelocity().getMagnitude());
        panelsTelemetry.debug("Turn rate (deg/s)", Math.toDegrees(follower.getAngularVelocity()));
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
