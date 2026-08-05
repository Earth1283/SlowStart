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
import org.firstinspires.ftc.teamcode.mechanisms.turret.MultiAxisTurret;
import org.firstinspires.ftc.teamcode.kernel.constants.autoConstants;
import org.firstinspires.ftc.teamcode.kernel.constants.robotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Team 32008 -- DECODE 2025-26 -- BLUE FAR autonomous.
 *
 * ============================================================================
 * Every tuning number this OpMode depends on now comes from 32008's own
 * competition-verified "FTC-32008 V2" repository. Nothing is borrowed from
 * another team and nothing is a placeholder. The PATH is the team's own
 * 8-segment route, unchanged.
 * ============================================================================
 *
 * AIM USES 32008'S TUNED CONSTANTS, NOT COMPUTED GEOMETRY.
 *
 * An earlier version of this file solved turret angle as (bearing - heading)
 * toward BLUE_TARGET. That is WRONG on this robot, and kernel/constants proves
 * it. Checked against 32008's verified values:
 *
 *   blue far: bearing-heading = -122.9 deg, but BLUE_FAR_TURRET = -70
 *   red far:  bearing-heading =  -10.0 deg, but RED_FAR_TURRET  = +70
 *
 * The offset needed to reconcile those is 52.9 deg for blue and 80.0 for red --
 * inconsistent, so the relationship is not geometric. The turret encoder is
 * zeroed wherever the turret sits at INIT, so these angles are hand-tuned
 * against that convention, not against the field frame. BLUE_TARGET is only
 * used for the TeleOp handoff in 32008's code; their autos never aim from it.
 *
 * Likewise FIRE DISTANCE IS NOT DISTANCE. FAR_FIRE_DISTANCE is 126.5 while the
 * true geometry from their shoot pose to the goal is 141.7 in. The gap absorbs
 * goal height, shooter offset and drag. Feeding real geometry into the velocity
 * curve overshoots by about 6% (1978 vs 1870 ticks/sec).
 *
 * The geometric solve is still computed and published to telemetry, purely so
 * it can be compared while tuning -- it does not drive anything.
 *
 * ROBUSTNESS -- an auto that freezes cannot even park, so every wait is bounded:
 *   - each path ends on complete OR isRobotStuck() OR PATH_TIMEOUT
 *   - each shot fires on at-speed OR SPINUP_TIMEOUT, never waits forever
 *   - a global PARK_DEADLINE abandons everything and runs the park path
 *   - activateAllPIDFs() in start(), because followPath() does NOT do it and a
 *     tuning OpMode may have left them off
 *
 * PATH -- the team's 8 segments split into 6 chains so the robot can stop and
 * shoot. Splitting also puts both near-reversals (corner 3->4 = 164.8 deg,
 * corner 7->8 = 177.4 deg) on chain boundaries where the robot fully stops; the
 * two corners it blends through mid-chain are the mildest at 40.6 and 68.1.
 *
 * TURRET ZERO: MultiAxisTurret zeroes its encoder at init, so every angle is
 * relative to where the turret physically sits when INIT is pressed. Park it the
 * same way every time.
 */
@Autonomous(name = "32008 Blue Far Auto", group = "32008")
@Configurable
public class BlueFarAuto extends OpMode {

    // ---- Path: 32008's VERIFIED start pose, the team's own route after that ----
    //
    // START comes from kernel autoConstants.BLUE_FAR_START -- (57.166, 7.362, 180 deg),
    // the pose 32008 actually starts blue far from. .copy() because that Pose is a
    // mutable static shared with every other auto; taking a reference would let one
    // OpMode's edit leak into another's start position.
    //
    // The drawn path's own start was (55.790, 8.210, 90 deg): 1.6 in away in position
    // (same physical spot) but 90 deg off in heading. Position was near enough to be
    // interchangeable; the HEADING was not, and the verified one wins because the
    // turret angles below are calibrated against it.
    private static final Pose START_POSE    = autoConstants.BLUE_FAR_START.copy();
    private static final Pose SHOOT_POSE    = new Pose(66.723, 18.970, Math.toRadians(120));
    private static final Pose MID_1_POSE    = new Pose(48.906, 34.497, Math.toRadians(180));
    private static final Pose PICKUP_1_POSE = new Pose(11.004, 34.805, Math.toRadians(180));
    private static final Pose SHOOT_2_POSE  = new Pose(67.108, 19.091, Math.toRadians(120));
    private static final Pose MID_2_POSE    = new Pose(48.862, 59.883, Math.toRadians(180));
    private static final Pose PICKUP_2_POSE = new Pose(14.918, 58.551, Math.toRadians(180));
    private static final Pose SHOOT_3_POSE  = new Pose(67.119, 19.121, Math.toRadians(120));
    private static final Pose PARK_POSE     = new Pose(57.735, 26.906, Math.toRadians(120));

    // ---- Bounds, all seconds. Every one exists to stop a hang. ----
    public static double PATH_TIMEOUT = 6.0;
    public static double SPINUP_TIMEOUT = 2.5;
    public static double PARK_DEADLINE = 25.0;
    /** Let the turret swing before firing, since it may travel 60+ degrees. */
    public static double TURRET_SETTLE_SECONDS = 0.35;

    // ---- Aim, from 32008's kernel autoConstants. Tunable live in Panels. ----
    /** Turret angle for the preload shot. 32008: BLUE_FAR_TURRET_PRELOAD. */
    public static double TURRET_PRELOAD_DEG = autoConstants.BLUE_FAR_TURRET_PRELOAD;
    /** Turret angle for every later shot. 32008: BLUE_FAR_TURRET. */
    public static double TURRET_SHOT_DEG = autoConstants.BLUE_FAR_TURRET;
    /** Tuned fire distance for the preload shot. 32008: FAR_FIRE_DISTANCE_PRELOAD. */
    public static double FIRE_DISTANCE_PRELOAD = autoConstants.FAR_FIRE_DISTANCE_PRELOAD;
    /** Tuned fire distance for every later shot. 32008: FAR_FIRE_DISTANCE. */
    public static double FIRE_DISTANCE = autoConstants.FAR_FIRE_DISTANCE;

    private enum State {
        DRIVE_TO_SHOOT_1, SHOOT_1,
        DRIVE_PICKUP_1,   DRIVE_TO_SHOOT_2, SHOOT_2,
        DRIVE_PICKUP_2,   DRIVE_TO_SHOOT_3, SHOOT_3,
        PARK, DONE
    }

    private Follower follower;
    private TelemetryManager panelsTelemetry;

    private final DualFlywheelShooter shooter = new DualFlywheelShooter();
    private final Intake intake = new RollerIntake();
    private final Gate   gate   = new DualServoGate();
    private final MultiAxisTurret turret = new MultiAxisTurret();

    private PathChain toShoot, pickup1, toShoot1, pickup2, toShoot2, park;

    private State state = State.DRIVE_TO_SHOOT_1;
    private final Timer stateTimer = new Timer();
    private final Timer opmodeTimer = new Timer();
    private final Timer feedTimer = new Timer();

    private boolean aimed = false;
    private boolean feeding = false;
    private String lastTransition = "none";
    private double aimTurretDeg = 0, aimVelocity = 0, aimHood = 0, aimFireDistance = 0;
    /** Telemetry only -- what pure geometry would say. Does not drive anything. */
    private double geoDistance = 0, geoTurretDeg = 0;
    private boolean preloadShotDone = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        gate.init(hardwareMap);
        turret.init(hardwareMap);
        gate.close();

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized -- BLUE FAR");
        panelsTelemetry.update(telemetry);
    }

    /** Keeps the localizer ticking before START so the pose is live and settled. */
    @Override
    public void init_loop() {
        follower.update();
        panelsTelemetry.debug("Status", "Ready -- park turret, place robot on LAUNCH LINE");
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    private void buildPaths() {
        // Interpolate FROM the real start heading, not the drawn one. START_POSE now
        // carries 180 deg, so hardcoding the drawn 90 deg here would tell the follower
        // to rotate from a heading the robot was never at -- it would spin 90 deg the
        // wrong way at launch trying to reach a start it had already passed.
        toShoot = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), Math.toRadians(120))
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
        // followPath() does NOT activate the PIDFs, and a tuning OpMode may have
        // switched them off. Without this the robot accepts paths and sits still.
        follower.activateAllPIDFs();

        // Idle the flywheel on the way out so the first shot spins up fast.
        shooter.hold();

        opmodeTimer.resetTimer();
        follower.followPath(toShoot, true);
        setState(State.DRIVE_TO_SHOOT_1, "start");
    }

    @Override
    public void loop() {
        follower.update();
        turret.update();

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
     * Commands shooter + turret from 32008's tuned constants.
     *
     * The preload shot gets its own pair because the robot has not moved from
     * the start yet; every later shot uses the standard pair.
     */
    private void aimAtGoal() {
        aimTurretDeg    = preloadShotDone ? TURRET_SHOT_DEG : TURRET_PRELOAD_DEG;
        aimFireDistance = preloadShotDone ? FIRE_DISTANCE   : FIRE_DISTANCE_PRELOAD;

        aimVelocity = RobotConstants.velocityForDistance(aimFireDistance);
        aimHood     = RobotConstants.hoodPercentForDistance(aimFireDistance);

        shooter.setTargetVelocity(aimVelocity);
        turret.setSetpoint(aimTurretDeg, aimHood);
        preloadShotDone = true;

        updateGeometryTelemetry();
    }

    /**
     * Pure geometry toward the blue goal. TELEMETRY ONLY -- published so the two
     * approaches can be compared on a real field. It is deliberately not wired
     * to the turret; see the class comment for why it does not agree with the
     * tuned values.
     */
    private void updateGeometryTelemetry() {
        Pose p = follower.getPose();
        double dx = robotConstants.BLUE_TARGET_X - p.getX();
        double dy = robotConstants.BLUE_TARGET_Y - p.getY();
        geoDistance = Math.hypot(dx, dy);
        double rel = Math.toDegrees(Math.atan2(dy, dx)) - Math.toDegrees(p.getHeading());
        while (rel > 180.0)  rel -= 360.0;
        while (rel < -180.0) rel += 360.0;
        geoTurretDeg = rel;
    }

    /** Path is done, stuck, or has taken too long. isBusy() alone hangs on a pinned robot. */
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
     * One volley. Aims, waits for the flywheel (bounded), feeds, closes up.
     * Fires on SPINUP_TIMEOUT regardless -- a slow shot may miss, a shot that
     * never happens blocks everything after it.
     */
    private boolean shotComplete() {
        if (!aimed) {
            aimAtGoal();
            aimed = true;
            return false;
        }

        if (!feeding) {
            boolean turretReady = stateTimer.getElapsedTimeSeconds() > TURRET_SETTLE_SECONDS;
            boolean atSpeed = shooter.atTargetVelocity();
            boolean waited = stateTimer.getElapsedTimeSeconds() > SPINUP_TIMEOUT;

            if ((turretReady && atSpeed) || waited) {
                gate.open();
                intake.fire();
                feeding = true;
                feedTimer.resetTimer();
                lastTransition = atSpeed ? "fired: at speed" : "fired: SPINUP TIMEOUT";
            }
            return false;
        }

        if (feedTimer.getElapsedTimeSeconds() > RobotConstants.GATE_FEED_SECONDS) {
            gate.close();
            intake.stop();
            shooter.hold();
            feeding = false;
            aimed = false;
            return true;
        }
        return false;
    }

    private void runStateMachine() {
        switch (state) {

            case DRIVE_TO_SHOOT_1:
                if (pathDone()) setState(State.SHOOT_1, "arrived: shoot 1");
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
                if (pathDone()) setState(State.SHOOT_2, "arrived: shoot 2");
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
                if (pathDone()) setState(State.SHOOT_3, "arrived: shoot 3");
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
                if (pathDone()) setState(State.DONE, "parked");
                break;

            case DONE:
            default:
                break;
        }
    }

    private void abortToPark() {
        gate.close();
        shooter.stop();
        intake.stop();
        feeding = false;
        aimed = false;
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
        panelsTelemetry.debug("Fire distance (tuned)", aimFireDistance);
        panelsTelemetry.debug("Aim turret (deg)", aimTurretDeg);
        panelsTelemetry.debug("Aim velocity", aimVelocity);
        panelsTelemetry.debug("Aim hood", aimHood);
        panelsTelemetry.debug("[ref] geometric dist", geoDistance);
        panelsTelemetry.debug("[ref] geometric turret", geoTurretDeg);
        panelsTelemetry.debug("Turret actual (deg)", turret.getYawDegrees());
        panelsTelemetry.debug("Shooter ticks/sec", shooter.getCurrentVelocity());
        panelsTelemetry.debug("Shooter at speed", shooter.atTargetVelocity());
        panelsTelemetry.debug("Gate open", gate.isOpen());
        panelsTelemetry.update(telemetry);
    }
}
