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
 * preload. Every heading interpolation is copied from the export exactly.
 *
 *   toShoot1  seg 1        start   -> shoot1    then SHOOT (preload)
 *   pickup1   segs 2 + 3   shoot1  -> pickup1   intake running
 *   toShoot2  seg 4        pickup1 -> shoot2    then SHOOT
 *   pickup2   segs 5 + 6   shoot2  -> pickup2   intake running
 *   toShoot3  seg 7        pickup2 -> shoot3    then SHOOT
 *   pickup3   segs 8 + 9   shoot3  -> pickup3   intake running
 *   toShoot4  seg 10       pickup3 -> shoot4    then SHOOT
 *
 * SHOT TRIGGER: the path is hand-drawn, so the four shoot poses are not identical
 * and the robot need not reach any of them exactly. A volley starts when the robot
 * comes within SHOOT_RADIUS of the ONE canonical shoot point and has slowed down,
 * OR when the leg ends -- whichever happens first. Getting near is enough.
 *
 * WHICH GOAL: this path shoots the BLUE goal, at about 36 in. Verified by checking
 * the same solve against 32008's own close-side constants: their BLUE_CLOSE_SHOOT
 * (54, 90) at heading 180 comes out at 66.53 in against their tuned
 * CLOSE_FIRE_DISTANCE of 68.5, and -46.55 deg against their tuned
 * BLUE_CLOSE_FIRE_TURRET of -47. Both land within 2 in / 0.5 deg.
 *
 * MECHANISMS ARE NOT GATED ON THE AIM SOLVE. The gate and intake run on plain
 * timers, and the flywheel is commanded every loop with a shooterHold() fallback.
 * An earlier version made the gate wait for aim lock, and one bad solve silently
 * killed the shooter, hood, gate and intake together.
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

    private static final Pose SHOOT_1_POSE  = new Pose(32.241, 108.327, Math.toRadians(130));
    private static final Pose MID_1_POSE    = new Pose(41.209,  82.786, Math.toRadians(180));
    private static final Pose PICKUP_1_POSE = new Pose(14.866,  82.820, Math.toRadians(180));

    private static final Pose SHOOT_2_POSE  = new Pose(32.060, 108.390, Math.toRadians(130));
    private static final Pose MID_2_POSE    = new Pose(43.208,  59.435, Math.toRadians(180));
    private static final Pose PICKUP_2_POSE = new Pose(18.837,  58.973, Math.toRadians(180));

    private static final Pose SHOOT_3_POSE  = new Pose(32.624, 107.879, Math.toRadians(130));
    private static final Pose MID_3_POSE    = new Pose(43.217,  35.190, Math.toRadians(180));
    private static final Pose PICKUP_3_POSE = new Pose(15.422,  35.251, Math.toRadians(180));

    private static final Pose SHOOT_4_POSE  = new Pose(32.475, 108.413, Math.toRadians(130));

    /** The one canonical shoot point. Proximity to THIS is what starts a volley. */
    public static double SHOOT_POINT_X = 32.24145616641902;
    public static double SHOOT_POINT_Y = 108.32689450222882;

    /** How near counts as "on the shoot point", inches. The path is hand-drawn. */
    public static double SHOOT_RADIUS = 6.0;
    /** ...and slow enough that the shot is not taken mid-sprint, inches/sec. */
    public static double SHOOT_SPEED_MAX = 6.0;

    /** BLUE goal, PEDRO frame. Must read 8 / 136 -- 136 / 136 is the RED goal. */
    public static double BLUE_GOAL_X = 144.0 - RobotConstants.BLUE_TARGET_Y;
    public static double BLUE_GOAL_Y = RobotConstants.BLUE_TARGET_X;

    /** Turret mounting trim, degrees, passed straight to AutoAim's yawOffset. */
    public static double YAW_OFFSET = 0.0;

    /**
     * Gate-open to fire delay, ms. AutoConstants.AUTO_CLOSE_WAIT_FOR_SHOOT is 0,
     * which would fire before the gate servos have physically travelled; their FAR
     * value of 400 is used instead. Drop it once timed on the real robot.
     */
    public static long WAIT_FOR_SHOOT_MS = 400;
    /** First volley waits longer -- the flywheel starts from cold. */
    public static long PRELOAD_EXTRA_MS = 800;
    public static long TOTAL_SHOOT_TIME_MS = 550;

    // Safety rails. Not from 32008 -- they keep a bad run from eating the period.
    // 406 in of path plus four volleys budgets ~18 s, so this is slack, not a cap.
    public static double PATH_TIMEOUT = 6.0;
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
        autoAim.init(hardwareMap);

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

    /** Every heading interpolation is the export's, verbatim. */
    private void buildPaths() {
        toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(130))
                .build();

        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_1_POSE, MID_1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .addPath(new BezierLine(MID_1_POSE, PICKUP_1_POSE))
                .setTangentHeadingInterpolation()
                .build();

        toShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_1_POSE, SHOOT_2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_2_POSE, MID_2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .addPath(new BezierLine(MID_2_POSE, PICKUP_2_POSE))
                .setTangentHeadingInterpolation()
                .build();

        toShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_2_POSE, SHOOT_3_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .build();

        pickup3 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_3_POSE, MID_3_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .addPath(new BezierLine(MID_3_POSE, PICKUP_3_POSE))
                .setTangentHeadingInterpolation()
                .build();

        toShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_3_POSE, SHOOT_4_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
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

        // Aim + shooter run EVERY loop, unconditionally. Nothing below is allowed
        // to depend on the state machine, and the state machine is not allowed to
        // depend on the aim solve.
        updateAim();
        driveShooter();
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

    /** Hand-drawn path: near the shoot point and slowed down is good enough. */
    private boolean nearShootPoint() {
        Pose p = follower.getPose();
        distanceToShootPoint = Math.hypot(SHOOT_POINT_X - p.getX(), SHOOT_POINT_Y - p.getY());
        return distanceToShootPoint <= SHOOT_RADIUS
                && follower.getVelocity().getMagnitude() <= SHOOT_SPEED_MAX;
    }

    /** True once the shooting leg is over: near enough, or the leg simply ended. */
    private boolean readyToShoot() {
        if (nearShootPoint()) {
            lastTransition = "at shoot point (" + String.format("%.1f", distanceToShootPoint) + " in)";
            return true;
        }
        return pathDone();
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

    /**
     * One volley, their order from BLUE_FAR_18 -- gate open, wait, fire, wait,
     * gate close. Pure timers: the aim solve is never consulted here.
     */
    private boolean shotComplete() {
        if (stateTimer.getElapsedTimeSeconds() > SHOOT_TIMEOUT) {
            intake.gateClose();
            intake.intakeStop();
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
                lastTransition = "shot: gate open, spinning up";
                return false;

            case 1: {
                long wait = WAIT_FOR_SHOOT_MS + (shotsFired == 0 ? PRELOAD_EXTRA_MS : 0);
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
                if (readyToShoot()) setState(State.SHOOT_1, "arrived: shoot 1");
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
                if (readyToShoot()) setState(State.SHOOT_2, "arrived: shoot 2");
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
                if (readyToShoot()) setState(State.SHOOT_3, "arrived: shoot 3");
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
                if (readyToShoot()) setState(State.SHOOT_4, "arrived: shoot 4");
                break;

            case SHOOT_4:
                if (shotComplete()) {
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
