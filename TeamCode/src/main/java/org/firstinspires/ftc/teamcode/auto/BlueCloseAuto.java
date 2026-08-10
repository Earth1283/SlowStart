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
 * preload. Segments 2, 7 and 10 are BezierCurves; the other seven are lines. Every
 * control point and heading interpolation is copied from the export exactly.
 *
 *   toShoot1  seg 1        start   -> shoot1    20.4 in   then SHOOT (preload, hdg 140)
 *   pickup1   seg 2 CURVE  shoot1  -> pickup1   73.5 in   hairpin, see SEG2_C1
 *             + seg 3
 *   toShoot2  seg 4        pickup1 -> shoot2    32.7 in   then SHOOT
 *   pickup2   segs 5 + 6   shoot2  -> pickup2   87.4 in
 *   toShoot3  seg 7 CURVE  pickup2 -> shoot3    72.4 in   then SHOOT -- CUSP, see SEG7
 *   pickup3   segs 8 + 9   shoot3  -> pickup3  117.5 in   longest leg, 25.5 in/s
 *   toShoot4  seg 10 CURVE pickup3 -> shoot4    84.1 in   then SHOOT (heading 140)
 *                                              ------- 487.9 in total
 *
 * SHOT TRIGGER: ARRIVAL, not proximity. The old version started a volley on
 * getting within SHOOT_RADIUS of a canonical point; that is no longer even
 * expressible, because the four shoot poses now sit 0.17 to 5.51 in apart -- being
 * at shoot 1 puts the robot inside any workable radius of shoots 2, 3 and 4 as
 * well. The leg ending IS the arrival signal now, which is also what "these are
 * approximate firing points, fire the moment you get there" actually means.
 *
 * THE VOLLEY FIRES ON MEASURED STILLNESS, not on a timer. Three things must be
 * true: the CHASSIS has actually stopped (STOPPED_SPEED_MAX / STOPPED_TURN_MAX read
 * off the follower), the turret is LOCKED, and the flywheel is AT SPEED. Then the
 * turret is PARKED (FREEZE_TURRET_ON_LOCK) so it cannot hunt, the per-shot settle
 * runs, and the shot goes. Only the gate's physical travel is still a plain timer,
 * because nothing on this robot senses gate position.
 *
 * Every one of those was learned the hard way. Waiting longer never fixed the
 * preload -- first because nothing checked whether the chassis had stopped coasting
 * (pathDone fires at t > 0.99, still moving), and then because the turret's own
 * static-friction term limit-cycles and never settles on its own.
 *
 * INTAKE RUNS THE WHOLE TIME, moving or stopped, from start() to the last shot.
 * It is commanded every loop next to the flywheel; the fire step just overrides
 * its power for the feed window.
 *
 * WHICH GOAL: the BLUE goal. All four shots now fire from 38.3 to 45.6 in, and
 * they have been walking steadily CLOSER every revision (64-69 in, then 45.5, then
 * 35-42, then 33-40, now 33-39.5). 32008's tuned CLOSE_FIRE_DISTANCE is 68.5 -- that is where their flywheel
 * and hood polynomials were fitted, and every shot here is now roughly half that.
 * Both curves still evaluate cleanly in range (1283-1308 rpm, hood 0.55-0.58, no
 * clipping, and 35 in is on the well-behaved increasing branch of the velocity
 * cubic, whose only turning point sits near 7.6 in). But this is extrapolation, and
 * it is the first thing to suspect if the close shots go long.
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

    // Heading 140 now, and NOTE THE DISCONTINUITY: the export ends seg 1 at 140 but
    // starts seg 2 at 130. Both are reproduced verbatim in buildPaths(), so the robot
    // holds 140 through the preload volley and then rotates 10 deg back as pickup1
    // begins. Harmless to the shot itself -- the aim solve reads the LIVE pose, not
    // this constant -- but it is a 10 deg jerk leaving the shoot point. Almost
    // certainly an editor slip; set seg 2 to start at 140 to remove it.
    private static final Pose SHOOT_1_POSE  = new Pose(38.339, 111.691, Math.toRadians(140));
    private static final Pose MID_1_POSE    = new Pose(45.211,  83.036, Math.toRadians(180));
    // The three PICKUP poses all sit hard against the x = 0 wall, and the commanded
    // X is deliberately deeper than an 18 in robot can physically reach:
    //   PICKUP_1  x = 12.97  ->  robot edge stops  +3.97 in short of the wall
    //   PICKUP_2  x =  9.05  ->                    +0.05 in  (flush)
    //   PICKUP_3  x =  6.20  ->                    -2.80 in  (2.8 in INSIDE the wall)
    // That is why these legs never report parametric end and run to their timeouts --
    // see INTAKE_TIMEOUT_1/2/3. It is intentional, not a bug: pushing into the ball
    // board is what collects. Just do not read the commanded pose as a reachable one.
    private static final Pose PICKUP_1_POSE = new Pose(12.973,  82.610, Math.toRadians(180));

    private static final Pose SHOOT_2_POSE  = new Pose(37.314, 104.466, Math.toRadians(130));
    private static final Pose MID_2_POSE    = new Pose(49.327,  58.933, Math.toRadians(180));
    private static final Pose PICKUP_2_POSE = new Pose( 9.046,  58.416, Math.toRadians(180));

    private static final Pose SHOOT_3_POSE  = new Pose(37.459, 104.308, Math.toRadians(130));
    private static final Pose MID_3_POSE    = new Pose(52.872,  35.203, Math.toRadians(180));
    private static final Pose PICKUP_3_POSE = new Pose( 6.202,  35.016, Math.toRadians(180));

    // NOTE heading 140, not 130 like the other three. Seg 10 is the only leg whose
    // interpolation target differs, and it has moved 130 -> 135 -> 140 over successive
    // revisions. buildPaths() below must match this number.
    private static final Pose SHOOT_4_POSE  = new Pose(44.809, 108.625, Math.toRadians(140));

    // Control points for the three BezierCurves, straight from the export.
    //
    // SEG 2 is still a hairpin -- its control point sits at x = 66.2, past its own
    // endpoint at x = 45.2, so the curve runs wide and turns back on itself -- but it
    // has RELAXED enough to stop being a problem. Tightest radius 3.0 in (was 1.8),
    // saturating centripetal above 21.0 in/s while its leg now needs only 19.9 in/s.
    // That inequality was the wrong way round for nine straight revisions; this is the
    // first export where seg 2 can make its schedule without running saturated.
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
    // Seg 10 is gentle by comparison -- 79 in minimum radius, nothing to watch.
    private static final Pose SEG10_C1 = new Pose(29.373, 55.786);

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

    /**
     * PER-SHOT SETTLE, seconds held AFTER the solve locks and the flywheel reaches
     * speed. NOT measured from arrival -- that is what changed.
     *
     * The arrival-based version was chasing the wrong thing. It went 0.5, then 0.9,
     * and the preload still wobbled, because the wobble was never about how long the
     * robot waited: followPath ran with holdEnd = true, so the chassis stayed under
     * active PIDF correction for the whole "wait". Waiting longer only wobbled longer.
     * The chassis is now genuinely parked first (see holdStill), AutoAim converges
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
     * robot is still MOVING when the leg is declared over. holdStill() then cuts the
     * drive command, but cutting power does not stop a robot, it lets it coast. So
     * the volley was starting during the coast-down: AutoAim was tracking a chassis
     * that was still drifting, the turret chased it, and the shot went off into that.
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
    /** Final row: +0.3 on top of the lock, same as shot 3. */
    public static double SETTLE_SHOOT_4 = 0.3;

    // Safety rails. Not from 32008 -- they keep a bad run from eating the period.
    // 444.5 in of path plus four volleys budgets ~15 s, so these are slack, not caps.
    public static double PATH_TIMEOUT = 7.0;
    /**
     * PER-LEG collection time, seconds. These are no longer bail-outs -- on this path
     * they ARE the leg duration, and the driver timed them.
     *
     * WHY: every pickup pose sits hard against the ball board (x = 16.1, 9.0, 6.2).
     * The robot physically cannot reach the commanded pose, so Pedro never reports
     * parametric end and never reports stuck either -- it just keeps pushing until
     * the timeout expires. Observed as "moves, headbutts the board, then waits there".
     * That waiting is what these numbers cut.
     *
     * From a flat 5.0 across all three, by the driver's own stopwatch:
     *   pickup1  5.0 - 1.3 = 3.7   (54.2 in leg -- 14.6 in/s to reach the board)
     *   pickup2  5.0 - 0.5 = 4.5   (84.6 in leg -- 18.8 in/s)
     *   pickup3  5.0 - 0.4 = 4.6   (113.0 in leg -- 24.6 in/s)
     *
     * Watch pickup3: 24.6 in/s is the tightest of the three, and unlike the other two
     * it has to cover a heading change on the way. If it starts arriving late, that
     * is the one to give time back to first.
     */
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
    /** Runs from the moment the solve LOCKS, not from arrival. Reset if lock drops. */
    private final Timer settleTimer = new Timer();
    private boolean wasReady = false;
    /** True once the turret has been parked for this volley. See FREEZE_TURRET_ON_LOCK. */
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

        // ALLIANCE HANDOFF. TeleOp (AASSTEST) never picks a colour -- it aims at
        // whatever these hold, so the auto is what decides. Their BLUE_FAR_18 sets
        // the same pair in loop() and stop(). Without this TeleOp aims at whatever
        // the last run left behind. Stated in the PINPOINT frame, which is the
        // frame TeleOp works in, so no conversion.
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
     * Hands the drivetrain to manual control with a zero command and BRAKE zero-power,
     * so the chassis is genuinely parked rather than being held by the path PIDF.
     *
     * Called once on entering a volley, then re-asserted every loop of it -- cheap,
     * and it means a stray path update cannot quietly take the wheels back.
     */
    private void holdStill() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
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
        // Re-assert the zero drive command for every loop of the volley. holdStill()
        // set the mode on phase 0; this keeps the commanded vector at zero.
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
                // STOP THE CHASSIS. Arriving at a shoot point did NOT stop it before:
                // followPath was given holdEnd = true, so the follower kept running its
                // PIDF to hold the endpoint, and the robot was still being actively
                // corrected -- and visibly wobbling -- while the shot went off. Adding
                // settle time never fixed that, because nothing was ever asking the
                // drivetrain to stop. This does.
                //
                // startTeleopDrive(true) = BRAKE zero-power behaviour, not FLOAT, so
                // the motors resist motion instead of coasting. Nothing needs undoing
                // afterwards: followPath() calls breakFollowing() internally, which
                // clears manualDrive on its own when the next leg starts.
                holdStill();
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
                shotPhase = 1;
                lastTransition = "shot: chassis held, gate opening";
                return false;

            case 1: {
                // Gate must have physically travelled first -- nothing senses it.
                if (shotTimer.getElapsedTime() < GATE_TRAVEL_MS) {
                    return false;
                }
                // AutoAim does its job HERE, with the chassis already held still.
                // The settle clock does not start until the solve is actually locked
                // and the flywheel is at speed -- and it RESTARTS if either drops out,
                // so a lock that flickers cannot sneak a shot through on a stale timer.
                boolean ready = readyToFire();
                if (ready && !wasReady) {
                    wasReady = true;
                    settleTimer.resetTimer();
                    aimErrorAtFreeze = aim.aimError;
                    if (FREEZE_TURRET_ON_LOCK) turretFrozen = true;
                    lastTransition = "shot: LOCKED, turret parked, settling " + settle + "s";
                } else if (!ready && wasReady) {
                    wasReady = false;
                    lastTransition = "shot: lock LOST, settle restarts";
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

            // ARRIVAL is the trigger, every time. No radius, no slow-down gate --
            // the four shoot poses are 0.17 to 5.51 in apart and no radius can tell
            // them apart, so the leg ending is what "we are there" means now.
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
