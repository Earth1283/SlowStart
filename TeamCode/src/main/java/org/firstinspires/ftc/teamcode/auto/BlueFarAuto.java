package org.firstinspires.ftc.teamcode.auto;

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

import org.firstinspires.ftc.teamcode.kernel.constants.autoConstants;
import org.firstinspires.ftc.teamcode.kernel.motion.GoTo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Team 32008 -- DECODE 2025-26 -- BLUE FAR autonomous.
 *
 * This is 32008's own auto/BLUE_FAR_18.java sequence, driving their own
 * subsystems/Shooter.java and subsystems/Intake.java (copied verbatim into
 * teamcode/subsystems). THE ONLY THING CHANGED IS THE PATH -- and the two
 * numbers the path forces to change, derived below.
 *
 * Their FTCLib SequentialCommandGroup is expressed as a plain timed state
 * machine because FTCLib is not a dependency of this repo. Same calls, same
 * order, same waits.
 *
 * WHY THE TURRET ANGLE IS NOT -70
 * -------------------------------
 * autoConstants.BLUE_FAR_TURRET = -70 is measured from THEIR shoot pose,
 * (59, 17) at heading 180 deg. The turret angle is RELATIVE TO THE ROBOT, so
 * rotating the shoot pose rotates the required angle with it. This path shoots
 * from (66.7, 19.0) at heading 120 deg, so -70 pointed ~65 deg off -- across the
 * field at the RED goal, which is the reported symptom.
 *
 * Re-solved for this path's shoot pose, against the blue goal in the PEDRO
 * frame. The kernel states goals in the PINPOINT frame (pinX = pedroY,
 * pinY = 144 - pedroX, per V2 tests/AASSTEST.java:59), so kernel BLUE_TARGET
 * (136, 136) is Pedro (8, 136); Pedro (136, 136) is the RED goal.
 *
 * The solve is validated by reproducing their own verified numbers:
 *
 *   their preload pose -> -70.03 deg solved vs -70.5 tuned   (0.47 off)
 *   their shoot pose   -> -67.79 deg solved vs -70.0 tuned   (2.21 off)
 *   their red pose     -> +67.79 deg solved vs +70.0 tuned   (2.21 off)
 *
 * and the same solve puts THIS path's shoot pose at 134.95 in range, which after
 * their own -8.46 in trim is 126.49 -- i.e. it independently reproduces
 * autoConstants.FAR_FIRE_DISTANCE = 126.5. That is the cross-check that says the
 * geometry is right rather than merely self-consistent.
 *
 * All three of this path's shoot poses are the same spot within 0.4 in, so one
 * turret angle and one distance cover all three shots.
 */
@Autonomous(name = "32008 Blue Far Auto", group = "32008")
@Configurable
public class BlueFarAuto extends OpMode {

    // 32008's start POSITION, this path's own start HEADING (90 deg).
    // Their 180 deg belongs to their path; mixing it in here rotates Pedro's whole
    // field frame 90 deg from reality and the robot drives sideways/backwards.
    private static final Pose START_POSE = new Pose(
            autoConstants.BLUE_FAR_START.getX(),
            autoConstants.BLUE_FAR_START.getY(),
            Math.toRadians(90));
    private static final Pose SHOOT_POSE    = new Pose(66.723, 18.970, Math.toRadians(120));
    private static final Pose MID_1_POSE    = new Pose(48.906, 34.497, Math.toRadians(180));
    private static final Pose PICKUP_1_POSE = new Pose(11.004, 34.805, Math.toRadians(180));
    private static final Pose SHOOT_2_POSE  = new Pose(67.108, 19.091, Math.toRadians(120));
    private static final Pose MID_2_POSE    = new Pose(48.862, 59.883, Math.toRadians(180));
    private static final Pose PICKUP_2_POSE = new Pose(14.918, 58.551, Math.toRadians(180));
    private static final Pose SHOOT_3_POSE  = new Pose(67.119, 19.121, Math.toRadians(120));
    private static final Pose PARK_POSE     = new Pose(57.735, 26.906, Math.toRadians(120));

    /**
     * Turret angle for THIS path's shoot pose, degrees, replacing their -70.
     * Solved -4.76; per-shot spread across the three shoot poses is 0.2 deg, so
     * one value covers all three. Trim here in Panels if shots pull left/right.
     */
    public static double BLUE_FAR_TURRET = -4.76;

    /** Their FAR_FIRE_DISTANCE. This path's geometry independently gives 126.49. */
    public static double FIRE_DISTANCE = autoConstants.FAR_FIRE_DISTANCE;
    /** Their FAR_HOLD_DISTANCE -- what the shooter idles at between volleys. */
    public static double HOLD_DISTANCE = autoConstants.FAR_HOLD_DISTANCE;

    // Their timings, unchanged. AUTO_FAR_WAIT_FOR_SHOOT is 400 ms in the kernel;
    // their preload gets +800 ms because the flywheel starts from cold.
    public static long WAIT_FOR_SHOOT_MS = autoConstants.AUTO_FAR_WAIT_FOR_SHOOT;
    public static long PRELOAD_EXTRA_MS = 800;
    public static long TOTAL_SHOOT_TIME_MS = 550;

    // Safety rails. Not from 32008 -- they keep a bad run from eating the period.
    public static double PATH_TIMEOUT = 6.0;
    public static double PARK_DEADLINE = 25.0;

    private enum State {
        DRIVE_TO_SHOOT_1, SHOOT_1,
        DRIVE_PICKUP_1,   DRIVE_TO_SHOOT_2, SHOOT_2,
        DRIVE_PICKUP_2,   DRIVE_TO_SHOOT_3, SHOOT_3,
        PARK, DONE
    }

    private Follower follower;
    private GoTo goTo;
    private TelemetryManager panelsTelemetry;

    // 32008's own subsystems, copied verbatim.
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();

    private PathChain pickup1, pickup2, park;

    private State state = State.DRIVE_TO_SHOOT_1;
    private final Timer stateTimer = new Timer();
    private final Timer opmodeTimer = new Timer();
    private final Timer shotTimer = new Timer();

    /** Drives the shooter every loop, exactly as their loop() does. */
    private double distance = autoConstants.FAR_HOLD_DISTANCE;

    private int shotPhase = 0;
    private boolean preloadDone = false;
    private String lastTransition = "none";

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        goTo = new GoTo(follower);

        // Their Robot.autoInit(): intake, shooter, then zero the turret encoder.
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        shooter.reset();

        intake.gateClose();
        distance = HOLD_DISTANCE;

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized -- BLUE FAR");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        follower.update();
        panelsTelemetry.debug("Status", "Ready -- park turret FORWARD, robot on LAUNCH LINE");
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Turret ticks", shooter.getTurretPosition());
        panelsTelemetry.debug("Turret deg", shooter.getTurretDegree());
        panelsTelemetry.update(telemetry);
    }

    private void buildPaths() {
        // toShoot, toShoot1, toShoot2 are single-segment, linear-heading legs,
        // so i removed them and used the kernel's goto API instead

        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE, MID_1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .addPath(new BezierLine(MID_1_POSE, PICKUP_1_POSE))
                .setTangentHeadingInterpolation()
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_2_POSE, MID_2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .addPath(new BezierLine(MID_2_POSE, PICKUP_2_POSE))
                .setTangentHeadingInterpolation()
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_3_POSE, PARK_POSE))
                .setTangentHeadingInterpolation()
                .build();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();

        // Point the turret now so it is settled by the time the robot arrives.
        shooter.turretToDegree(BLUE_FAR_TURRET);

        opmodeTimer.resetTimer();
        goTo.goTo(START_POSE, SHOOT_POSE);
        setState(State.DRIVE_TO_SHOOT_1, "start");
    }

    @Override
    public void loop() {
        follower.update();

        // Their loop() does exactly this every iteration: the flywheel AND the hood
        // are commanded continuously from `distance`. Nothing moves if this is not
        // called every loop.
        shooter.setShooterByDis(distance);

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
        shooter.shooterStop();
        intake.intakeStop();
        intake.gateClose();
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
     * One volley, their exact order from BLUE_FAR_18:
     *
     *   distance = FIRE_DISTANCE; turretToDegree; gateOpen; intakeEngage
     *   wait AUTO_FAR_WAIT_FOR_SHOOT   (preload waits +800)
     *   intakeFire(calculateIntakePower())
     *   wait TOTAL_SHOOT_TIME
     *   gateClose; intakeDisengage; distance = HOLD_DISTANCE
     */
    private boolean shotComplete() {
        switch (shotPhase) {

            case 0:
                distance = FIRE_DISTANCE;
                shooter.turretToDegree(BLUE_FAR_TURRET);
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
                    distance = HOLD_DISTANCE;
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
                    goTo.goTo(PICKUP_1_POSE, SHOOT_2_POSE);
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
                    goTo.goTo(PICKUP_2_POSE, SHOOT_3_POSE);
                    setState(State.DRIVE_TO_SHOOT_3, "pickup 2 done");
                }
                break;

            case DRIVE_TO_SHOOT_3:
                if (pathDone()) setState(State.SHOOT_3, "arrived: shoot 3");
                break;

            case SHOOT_3:
                if (shotComplete()) {
                    intake.intakeStop();
                    follower.followPath(park, true);
                    setState(State.PARK, "shot 3 done");
                }
                break;

            case PARK:
                if (pathDone()) {
                    distance = 0;
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
        distance = HOLD_DISTANCE;
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

        panelsTelemetry.debug("distance (cmd)", distance);
        panelsTelemetry.debug("Turret cmd (deg)", BLUE_FAR_TURRET);
        panelsTelemetry.debug("Turret ticks", shooter.getTurretPosition());
        panelsTelemetry.debug("Turret actual (deg)", shooter.getTurretDegree());
        panelsTelemetry.debug("Turret busy", shooter.turret.isBusy());
        panelsTelemetry.debug("Hood percent", shooter.getHoodPercent());
        panelsTelemetry.debug("Shooter target", Shooter.targetVelocity);
        panelsTelemetry.debug("Shooter actual", shooter.getShooterVelocity());
        panelsTelemetry.debug("Shooter ready", shooter.shooterReady());
        panelsTelemetry.debug("Intake fire power", shooter.calculateIntakePower());
        panelsTelemetry.update(telemetry);
    }
}
