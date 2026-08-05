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

import org.firstinspires.ftc.teamcode.mechanisms.gate.Gate;
import org.firstinspires.ftc.teamcode.mechanisms.gate.DualServoGate;
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

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer;
    private int pathState;

    private Shooter shooter;
    private Intake intake;
    private Gate gate;
    private Turret turret;

    // -- Field poses, straight from the team's supplied path (inches, Pedro frame) --
    private static final Pose START_POSE   = new Pose(55.790, 8.210, Math.toRadians(90));
    private static final Pose SHOOT_POSE   = new Pose(66.723, 18.970, Math.toRadians(120));
    private static final Pose MID_1_POSE   = new Pose(48.906, 34.497, Math.toRadians(180));
    private static final Pose PICKUP_1_POSE = new Pose(11.004, 34.805, Math.toRadians(180));
    private static final Pose SHOOT_2_POSE = new Pose(67.108, 19.091, Math.toRadians(120));
    private static final Pose MID_2_POSE   = new Pose(48.862, 59.883, Math.toRadians(180));
    private static final Pose PICKUP_2_POSE = new Pose(14.918, 58.551, Math.toRadians(180));
    private static final Pose SHOOT_3_POSE = new Pose(67.119, 19.121, Math.toRadians(120));
    private static final Pose PARK_POSE    = new Pose(57.735, 26.906, Math.toRadians(120));

    private PathChain toShoot, pickup1, toShoot1, pickup2, toShoot2, park;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        // Matches the first path's own start point. The original code set (72, 8),
        // ~16 inches off the path start, which makes Pedro snap sideways at launch.
        follower.setStartingPose(START_POSE);

        shooter = new DualFlywheelShooter();
        intake = new RollerIntake();
        gate = new DualServoGate();
        turret = new MultiAxisTurret();

        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        gate.init(hardwareMap);
        turret.init(hardwareMap);

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Alliance", "BLUE FAR");
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
        turret.setSetpoint(RobotConstants.TURRET_PARK_YAW_DEG,
                           RobotConstants.TURRET_PARK_PITCH_PERCENT);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        turret.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Shooter ticks/sec", shooter.getCurrentVelocity());
        panelsTelemetry.debug("Shooter at speed", shooter.atTargetVelocity());
        panelsTelemetry.debug("Gate open", gate.isOpen());
        panelsTelemetry.debug("Turret yaw deg", turret.getYawDegrees());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        turret.stop();
    }

    /**
     * State machine. Returns void: the original returned an int that was assigned
     * back into pathState every loop, which resets the machine to 0 forever.
     */
    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:  // drive to the shooting pose, spinning the flywheel up on the way
                shooter.setTargetVelocity(RobotConstants.SHOOTER_TARGET_TICKS_PER_SEC);
                follower.followPath(toShoot, true);
                setPathState(1);
                break;

            case 1:  // SHOT 1 (preload) -- after segment 1
                if (!follower.isBusy() && shooter.atTargetVelocity()) {
                    gate.open();
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > RobotConstants.GATE_FEED_SECONDS) {
                    gate.close();
                    intake.intake();
                    follower.followPath(pickup1, true);
                    setPathState(3);
                }
                break;

            case 3:  // collected -- head back to shoot
                if (!follower.isBusy()) {
                    intake.stop();
                    follower.followPath(toShoot1, true);
                    setPathState(4);
                }
                break;

            case 4:  // SHOT 2 -- after segment 4
                if (!follower.isBusy() && shooter.atTargetVelocity()) {
                    gate.open();
                    setPathState(5);
                }
                break;

            case 5:
                if (pathTimer.getElapsedTimeSeconds() > RobotConstants.GATE_FEED_SECONDS) {
                    gate.close();
                    intake.intake();
                    follower.followPath(pickup2, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    intake.stop();
                    follower.followPath(toShoot2, true);
                    setPathState(7);
                }
                break;

            case 7:  // SHOT 3 -- after segment 7
                if (!follower.isBusy() && shooter.atTargetVelocity()) {
                    gate.open();
                    setPathState(8);
                }
                break;

            case 8:
                if (pathTimer.getElapsedTimeSeconds() > RobotConstants.GATE_FEED_SECONDS) {
                    gate.close();
                    shooter.stop();
                    intake.stop();
                    follower.followPath(park, true);
                    setPathState(9);
                }
                break;

            case 9:  // parked
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }
}
