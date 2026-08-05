package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.gate.DualServoGate;
import org.firstinspires.ftc.teamcode.mechanisms.gate.Gate;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.RollerIntake;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.DualFlywheelShooter;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.turret.MultiAxisTurret;
import org.firstinspires.ftc.teamcode.mechanisms.turret.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "32008 Blue Far Auto", group = "32008")
@Configurable
public class BlueFarAuto extends OpMode {

    private static final Pose START_POSE    = new Pose(55.790,  8.210, Math.toRadians(90));
    private static final Pose SHOOT_POSE    = new Pose(66.723, 18.970, Math.toRadians(120));
    private static final Pose MID_1_POSE    = new Pose(48.906, 34.497, Math.toRadians(180));
    private static final Pose PICKUP_1_POSE = new Pose(11.004, 34.805, Math.toRadians(180));
    private static final Pose SHOOT_2_POSE  = new Pose(67.108, 19.091, Math.toRadians(120));
    private static final Pose MID_2_POSE    = new Pose(48.862, 59.883, Math.toRadians(180));
    private static final Pose PICKUP_2_POSE = new Pose(14.918, 58.551, Math.toRadians(180));
    private static final Pose SHOOT_3_POSE  = new Pose(67.119, 19.121, Math.toRadians(120));
    private static final Pose PARK_POSE     = new Pose(57.735, 26.906, Math.toRadians(120));

    public static double PATH_TIMEOUT = 6.0;
    public static double SPINUP_TIMEOUT = 2.5;
    public static double PARK_DEADLINE = 25.0;

    private enum State {
        DRIVE_TO_SHOOT_1, SHOOT_1,
        DRIVE_PICKUP_1,   DRIVE_TO_SHOOT_2, SHOOT_2,
        DRIVE_PICKUP_2,   DRIVE_TO_SHOOT_3, SHOOT_3,
        PARK, DONE
    }

    private Follower follower;
    private TelemetryManager panelsTelemetry;

    private final Shooter shooter = new DualFlywheelShooter();
    private final Intake  intake  = new RollerIntake();
    private final Gate    gate    = new DualServoGate();
    private final Turret  turret  = new MultiAxisTurret();

    private PathChain toShoot, pickup1, toShoot1, pickup2, toShoot2, park;

    private State state = State.DRIVE_TO_SHOOT_1;
    private final Timer stateTimer = new Timer();
    private final Timer opmodeTimer = new Timer();

    private boolean feeding = false;
    private final Timer feedTimer = new Timer();
    private String lastTransition = "none";

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        gate.init(hardwareMap);
        turret.init(hardwareMap);

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized -- BLUE FAR");
        panelsTelemetry.update(telemetry);
    }
    @Override
    public void init_loop() {
        follower.update();
        panelsTelemetry.debug("Status", "Ready -- place robot on the LAUNCH LINE");
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    private void buildPaths() {
        toShoot = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                .build();

        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE, MID_1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .addPath(new BezierLine(MID_1_POSE, PICKUP_1_POSE))
                .setTangentHeadingInterpolation()
                .build();

        toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_1_POSE, SHOOT_2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_2_POSE, MID_2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .addPath(new BezierLine(MID_2_POSE, PICKUP_2_POSE))
                .setTangentHeadingInterpolation()
                .build();

        toShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_2_POSE, SHOOT_3_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_3_POSE, PARK_POSE))
                .setTangentHeadingInterpolation()
                .build();
    }

    @Override
    public void start() {
        // Defect 1: followPath() does not do this, and a tuning OpMode may have
        // switched the PIDFs off. Without it the robot accepts paths and sits still.
        follower.activateAllPIDFs();

        turret.setSetpoint(RobotConstants.TURRET_PARK_YAW_DEG,
                           RobotConstants.TURRET_PARK_PITCH_PERCENT);

        // Spin up on the way to the first shot rather than after arriving.
        shooter.setTargetVelocity(RobotConstants.SHOOTER_TARGET_TICKS_PER_SEC);

        opmodeTimer.resetTimer();
        follower.followPath(toShoot, true);
        setState(State.DRIVE_TO_SHOOT_1, "start");
    }

    @Override
    public void loop() {
        follower.update();
        turret.update();

        // Global deadline. Whatever is happening, get to the park attempt with
        // enough time left to finish it -- parking is worth real points and a
        // frozen state machine cannot claim them.
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
        shooter.stop();
        intake.stop();
        turret.stop();
    }

    /**
     * True once the current path is done, stuck, or has taken too long.
     *
     * Three-way like 19859's FollowPathCommand.isFinished(), plus a timeout.
     * isBusy() alone hangs forever on a robot pinned against a wall.
     */
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

    private void runStateMachine() {
        switch (state) {

            case DRIVE_TO_SHOOT_1:
                if (pathDone()) {
                    beginShot(State.SHOOT_1);
                }
                break;

            case SHOOT_1:
                if (shotComplete()) {
                    intake.intake();
                    follower.followPath(pickup1, true);
                    setState(State.DRIVE_PICKUP_1, "shot 1 done");
                }
                break;

            case DRIVE_PICKUP_1:
                if (pathDone()) {
                    intake.stop();
                    follower.followPath(toShoot1, true);
                    setState(State.DRIVE_TO_SHOOT_2, "pickup 1 done");
                }
                break;

            case DRIVE_TO_SHOOT_2:
                if (pathDone()) {
                    beginShot(State.SHOOT_2);
                }
                break;

            case SHOOT_2:
                if (shotComplete()) {
                    intake.intake();
                    follower.followPath(pickup2, true);
                    setState(State.DRIVE_PICKUP_2, "shot 2 done");
                }
                break;

            case DRIVE_PICKUP_2:
                if (pathDone()) {
                    intake.stop();
                    follower.followPath(toShoot2, true);
                    setState(State.DRIVE_TO_SHOOT_3, "pickup 2 done");
                }
                break;

            case DRIVE_TO_SHOOT_3:
                if (pathDone()) {
                    beginShot(State.SHOOT_3);
                }
                break;

            case SHOOT_3:
                if (shotComplete()) {
                    shooter.stop();
                    intake.stop();
                    follower.followPath(park, true);
                    setState(State.PARK, "shot 3 done");
                }
                break;

            case PARK:
                if (pathDone()) {
                    setState(State.DONE, "parked");
                }
                break;

            case DONE:
            default:
                break;
        }
    }

    private void beginShot(State shotState) {
        feeding = false;
        setState(shotState, "arrived at shoot pose");
    }

    /**
     * Runs one volley and reports when it is finished.
     *
     * Waits for the flywheel, but ONLY up to SPINUP_TIMEOUT -- then it fires
     * regardless. A shot at slightly-low speed may miss; a shot that never
     * happens guarantees the rest of the auto never happens either.
     */
    private boolean shotComplete() {
        if (!feeding) {
            boolean atSpeed = shooter.atTargetVelocity();
            boolean waitedLongEnough = stateTimer.getElapsedTimeSeconds() > SPINUP_TIMEOUT;
            if (atSpeed || waitedLongEnough) {
                gate.open();
                feeding = true;
                feedTimer.resetTimer();
                lastTransition = atSpeed ? "fired: at speed" : "fired: SPINUP TIMEOUT";
            }
            return false;
        }

        if (feedTimer.getElapsedTimeSeconds() > RobotConstants.GATE_FEED_SECONDS) {
            gate.close();
            feeding = false;
            return true;
        }
        return false;
    }

    /** Global-deadline escape: stop everything scoring-related and run the park path. */
    private void abortToPark() {
        gate.close();
        shooter.stop();
        intake.stop();
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
        panelsTelemetry.debug("Last transition", lastTransition);
        panelsTelemetry.debug("State time (s)", stateTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("Auto time (s)", opmodeTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Follower busy", follower.isBusy());
        panelsTelemetry.debug("Robot stuck", follower.isRobotStuck());
        panelsTelemetry.debug("Shooter ticks/sec", shooter.getCurrentVelocity());
        panelsTelemetry.debug("Shooter at speed", shooter.atTargetVelocity());
        panelsTelemetry.debug("Gate open", gate.isOpen());
        panelsTelemetry.debug("Turret yaw (deg)", turret.getYawDegrees());
        panelsTelemetry.update(telemetry);
    }
}
