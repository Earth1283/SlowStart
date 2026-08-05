package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.kernel.constants.autoConstants;
import org.firstinspires.ftc.teamcode.kernel.constants.robotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoAimSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "32008 Blue Far Auto", group = "32008")
@Configurable
public class BlueFarAuto extends OpMode {

    private static final Pose START_POSE    = new Pose(56.000,  8.000, Math.toRadians(90));

    private static final Pose SHOOT_1_POSE  = new Pose(63.569, 16.026, Math.toRadians(120));
    private static final Pose MID_1_POSE    = new Pose(48.418, 36.139, Math.toRadians(180));
    private static final Pose PICKUP_1_POSE = new Pose(10.641, 35.271, Math.toRadians(180));

    private static final Pose SHOOT_2_POSE  = new Pose(63.783, 16.023, Math.toRadians(120));
    private static final Pose MID_2_POSE    = new Pose(12.513, 23.225, Math.toRadians(100));
    // Heading here is documentation only -- BezierLine uses just the x/y of a Pose,
    // and segment 7 declares its own 60 deg start. Set to match that.
    private static final Pose PICKUP_2_POSE = new Pose( 8.033,  8.307, Math.toRadians(60));

    private static final Pose SHOOT_3_POSE  = new Pose(63.894, 15.991, Math.toRadians(120));
    private static final Pose MID_3_POSE    = new Pose(39.397, 41.491, Math.toRadians(180));
    private static final Pose PICKUP_3_POSE = new Pose(10.115, 23.550, Math.toRadians(180));

    private static final Pose SHOOT_4_POSE  = new Pose(63.506, 15.738, Math.toRadians(120));
    private static final Pose PARK_POSE     = new Pose(56.781, 22.746, Math.toRadians(120));

    public static double BLUE_GOAL_X = 144.0 - robotConstants.BLUE_TARGET_Y;
    public static double BLUE_GOAL_Y = robotConstants.BLUE_TARGET_X;

    public static double YAW_OFFSET = 0.0;

    // Their timings, unchanged. AUTO_FAR_WAIT_FOR_SHOOT is 400 ms in the kernel;
    // the first volley gets +800 ms because the flywheel starts from cold.
    public static long WAIT_FOR_SHOOT_MS = autoConstants.AUTO_FAR_WAIT_FOR_SHOOT;
    public static long PRELOAD_EXTRA_MS = 800;
    public static long TOTAL_SHOOT_TIME_MS = 550;

    // Safety rails. Not from 32008 -- they keep a bad run from eating the period.
    public static double PATH_TIMEOUT = 6.0;
    public static double PARK_DEADLINE = 26.0;

    private enum State {
        DRIVE_TO_SHOOT_1, SHOOT_1,
        DRIVE_PICKUP_1,   DRIVE_TO_SHOOT_2, SHOOT_2,
        DRIVE_PICKUP_2,   DRIVE_TO_SHOOT_3, SHOOT_3,
        DRIVE_PICKUP_3,   DRIVE_TO_SHOOT_4, SHOOT_4,
        PARK, DONE
    }

    private Follower follower;
    private TelemetryManager panelsTelemetry;

    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final AutoAimSubsystem autoAim = new AutoAimSubsystem();

    private AutoAimSubsystem.TurretCommand aim = new AutoAimSubsystem.TurretCommand();

    private PathChain toShoot1, pickup1, toShoot2, pickup2,
                     toShoot3, pickup3, toShoot4, park;

    private State state = State.DRIVE_TO_SHOOT_1;
    private final Timer stateTimer = new Timer();
    private final Timer opmodeTimer = new Timer();
    private final Timer shotTimer = new Timer();

    private int shotPhase = 0;
    private boolean preloadDone = false;
    private boolean shooterLive = false;
    private String lastTransition = "none";

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        intake.init(hardwareMap);
        // aass = true: AutoAim owns turret "lt" AND hood "panel". Without this
        // both classes grab the turret and fight over its run mode.
        shooter.init(hardwareMap, true);
        autoAim.init(hardwareMap);

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

    private void buildPaths() {
        toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                .build();

        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_1_POSE, MID_1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .addPath(new BezierLine(MID_1_POSE, PICKUP_1_POSE))
                .setTangentHeadingInterpolation()
                .build();

        toShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_1_POSE, SHOOT_2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_2_POSE, MID_2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(100))
                .addPath(new BezierLine(MID_2_POSE, PICKUP_2_POSE))
                .setTangentHeadingInterpolation()
                .build();
        toShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_2_POSE, SHOOT_3_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(120))
                .build();

        pickup3 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_3_POSE, MID_3_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .addPath(new BezierLine(MID_3_POSE, PICKUP_3_POSE))
                .setTangentHeadingInterpolation()
                .build();

        toShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_3_POSE, SHOOT_4_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_4_POSE, PARK_POSE))
                .setTangentHeadingInterpolation()
                .build();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();

        shooterLive = true;

        opmodeTimer.resetTimer();
        follower.followPath(toShoot1, true);
        setState(State.DRIVE_TO_SHOOT_1, "start");
    }

    @Override
    public void loop() {
        follower.update();

        updateAim();
        driveShooter();
        publishPoseForTeleOp();

        if (opmodeTimer.getElapsedTimeSeconds() > PARK_DEADLINE
                && state != State.PARK && state != State.DONE) {
            abortToPark();
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

    private void publishPoseForTeleOp() {
        Pose p = follower.getPose();
        robotConstants.autoEndX = p.getX();
        robotConstants.autoEndY = p.getY();
        robotConstants.autoEndH = p.getHeading();
    }

    private void updateAim() {
        Pose p = follower.getPose();
        Vector v = follower.getVelocity();
        double headingDeg = Math.toDegrees(p.getHeading());
        double shooterX = p.getX() + Math.cos(p.getHeading()) * robotConstants.SHOOTER_DRIVETRAIN_OFFSET;
        double shooterY = p.getY() + Math.sin(p.getHeading()) * robotConstants.SHOOTER_DRIVETRAIN_OFFSET;

        aim = autoAim.update(
                shooterX, shooterY,
                v.getXComponent(), v.getYComponent(),
                headingDeg,
                // Pedro reports heading rate in RADIANS/sec; AutoAim wants degrees.
                Math.toDegrees(follower.getAngularVelocity()),
                BLUE_GOAL_X, BLUE_GOAL_Y,
                false, 0.0,
                false,
                !follower.isBusy(),
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

    private boolean pathDone() {
        if (follower.isRobotStuck()) {
            lastTransition = "path ended: ROBOT STUCK";
            return true;
        }
        if (stateTimer.getElapsedTimeSeconds() > PATH_TIMEOUT) {
            lastTransition = "path ended: TIMEOUT";
            return true;
        }
        if (!follower.isBusy() || follower.atParametricEnd()) {
            lastTransition = "path ended: complete";
            return true;
        }
        return false;
    }

    private boolean shotComplete() {
        switch (shotPhase) {

            case 0:
                intake.gateOpen();
                intake.intakeEngage();
                shotTimer.resetTimer();
                shotPhase = 1;
                lastTransition = "shot: gate open, spinning up";
                return false;

            case 1: {
                long wait = WAIT_FOR_SHOOT_MS + (preloadDone ? 0 : PRELOAD_EXTRA_MS);
                if (shotTimer.getElapsedTime() >= wait) {
                    intake.intakeFire(shooter.calculateIntakePower());
                    shotTimer.resetTimer();
                    shotPhase = 2;
                    lastTransition = "shot: FIRING";
                }
                return false;
            }

            case 2:
                if (shotTimer.getElapsedTime() >= TOTAL_SHOOT_TIME_MS) {
                    intake.gateClose();
                    intake.intakeDisengage();
                    intake.intakeStop();
                    preloadDone = true;
                    shotPhase = 0;
                    lastTransition = "shot: done";
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
                if (pathDone()) setState(State.SHOOT_1, "arrived: shoot 1");
                break;

            case SHOOT_1:
                if (shotComplete()) {
                    intake.intakeIn();
                    follower.followPath(pickup1, true);
                    setState(State.DRIVE_PICKUP_1, "shot 1 done");
                }
                break;

            case DRIVE_PICKUP_1:
                if (pathDone()) {
                    intake.intakeStop();
                    follower.followPath(toShoot2, true);
                    setState(State.DRIVE_TO_SHOOT_2, "pickup 1 done");
                }
                break;

            case DRIVE_TO_SHOOT_2:
                if (pathDone()) setState(State.SHOOT_2, "arrived: shoot 2");
                break;

            case SHOOT_2:
                if (shotComplete()) {
                    intake.intakeIn();
                    follower.followPath(pickup2, true);
                    setState(State.DRIVE_PICKUP_2, "shot 2 done");
                }
                break;

            case DRIVE_PICKUP_2:
                if (pathDone()) {
                    intake.intakeStop();
                    follower.followPath(toShoot3, true);
                    setState(State.DRIVE_TO_SHOOT_3, "pickup 2 done");
                }
                break;

            case DRIVE_TO_SHOOT_3:
                if (pathDone()) setState(State.SHOOT_3, "arrived: shoot 3");
                break;

            case SHOOT_3:
                if (shotComplete()) {
                    intake.intakeIn();
                    follower.followPath(pickup3, true);
                    setState(State.DRIVE_PICKUP_3, "shot 3 done");
                }
                break;

            case DRIVE_PICKUP_3:
                if (pathDone()) {
                    intake.intakeStop();
                    follower.followPath(toShoot4, true);
                    setState(State.DRIVE_TO_SHOOT_4, "pickup 3 done");
                }
                break;

            case DRIVE_TO_SHOOT_4:
                if (pathDone()) setState(State.SHOOT_4, "arrived: shoot 4");
                break;

            case SHOOT_4:
                if (shotComplete()) {
                    intake.intakeStop();
                    follower.followPath(park, true);
                    setState(State.PARK, "shot 4 done");
                }
                break;

            case PARK:
                if (pathDone()) {
                    shooterLive = false;
                    shooter.shooterStop();
                    setState(State.DONE, "parked");
                }
                break;

            case DONE:
            default:
                break;
        }
    }

    private void abortToPark() {
        intake.gateClose();
        intake.intakeStop();
        shotPhase = 0;
        follower.followPath(park, true);
        setState(State.PARK, "ABORT: park deadline reached");
    }

    private void setState(State next, String why) {
        state = next;
        lastTransition = why;
        stateTimer.resetTimer();
    }

    private void report() {
        panelsTelemetry.debug("State", state);
        panelsTelemetry.debug("Shot phase", shotPhase);
        panelsTelemetry.debug("Last transition", lastTransition);
        panelsTelemetry.debug("State time (s)", stateTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("Auto time (s)", opmodeTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Follower busy", follower.isBusy());
        panelsTelemetry.debug("Robot stuck", follower.isRobotStuck());

        panelsTelemetry.debug("Aim has target", aim.hasTarget);
        panelsTelemetry.debug("Aim LOCKED", aim.isAimLocked);
        panelsTelemetry.debug("Aim range (in)", aim.targetDist);
        panelsTelemetry.debug("Aim error (deg)", aim.aimError);
        panelsTelemetry.debug("Aim tolerance (deg)", aim.currentTolerance);
        panelsTelemetry.debug("Aim flight time (s)", aim.flightTime);
        panelsTelemetry.debug("Turret target (deg)", aim.targetTurretAngle);
        panelsTelemetry.debug("Turret actual (deg)", autoAim.getCurrentTurretAngle());
        panelsTelemetry.debug("Turret ticks", autoAim.getCurrentTick());

        panelsTelemetry.debug("HOOD cmd (percent)", aim.targetPitch);
        panelsTelemetry.debug("HOOD servo pos", autoAim.hood.getPosition());
        panelsTelemetry.debug("Shooter target", aim.targetRpm);
        panelsTelemetry.debug("Shooter actual", shooter.getShooterVelocity());
        panelsTelemetry.debug("Shooter ready", shooter.shooterReady(aim.targetRpm));
        panelsTelemetry.debug("Intake fire power", shooter.calculateIntakePower());
        panelsTelemetry.debug("Battery (V)", autoAim.getCurrentBatteryVoltage());
        panelsTelemetry.update(telemetry);
    }
}
