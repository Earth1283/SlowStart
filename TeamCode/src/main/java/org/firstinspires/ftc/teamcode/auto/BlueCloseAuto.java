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

// STOP WITH THE STUPID JAVADOCS

@Autonomous(name = "32008 Blue Close Auto", group = "32008")
@Configurable
public class BlueCloseAuto extends OpMode {
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
    private static final Pose SEG7_C1 = new Pose(64.430, 55.334);
    private static final Pose SEG7_C2 = new Pose(36.084, 99.105);
    private static final Pose SEG10_C1 = new Pose(24.327, 60.832);

    public static double MAX_POWER = 1.0;

    public static double BRAKING_STRENGTH = 1.0;
    public static double BRAKING_START = 1.0;

    /** BLUE goal, PEDRO frame. Must read 8 / 136 -- 136 / 136 is the RED goal. */
    public static double BLUE_GOAL_X = 144.0 - RobotConstants.BLUE_TARGET_Y;
    public static double BLUE_GOAL_Y = RobotConstants.BLUE_TARGET_X;

    /** Turret mounting trim, degrees, passed straight to AutoAim's yawOffset. */
    public static double YAW_OFFSET = 0.0;

    public static long GATE_TRAVEL_MS = 400;
    public static long TOTAL_SHOOT_TIME_MS = 550;
    public static double READY_TIMEOUT = 1.2;

    // Safety rails. Not from 32008 -- they keep a bad run from eating the period.
    // 444.5 in of path plus four volleys budgets ~15 s, so these are slack, not caps.
    public static double PATH_TIMEOUT = 7.0;
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
        RobotConstants.autoEndH = p.getHeading
        RobotConstants.teleOpTargetX = RobotConstants.BLUE_TARGET_X;
        RobotConstants.teleOpTargetY = RobotConstants.BLUE_TARGET_Y;
    }

    private void updateAim() {
        Pose p = follower.getPose();
        Vector v = follower.getVelocity();
        double headingDeg = Math.toDegrees(p.getHeading());
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
