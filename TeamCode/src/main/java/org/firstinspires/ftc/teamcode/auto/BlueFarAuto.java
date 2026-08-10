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
 * Team 32008 -- DECODE 2025-26 -- BLUE FAR autonomous.
 *
 * Same subsystems and the same volley machinery as {@link BlueCloseAuto}: 32008's
 * own {@link AutoAimSubsystem}, {@link Shooter} and {@link Intake}, copied verbatim.
 * Only the PATH and the far-range shot timing are this team's own.
 *
 * PATH: the 10-segment Pedro Pathing export, split into 7 chains so the robot can
 * stop and shoot. FOUR volleys, after segments 1, 4, 7 and 10 -- the first is the
 * preload. Segment 8 is the only BezierCurve; the other nine are lines.
 *
 *   toShoot1  seg 1        start   -> shoot1    14.6 in   then SHOOT (preload)
 *   pickup1   segs 2 + 3   shoot1  -> pickup1   55.5 in
 *   toShoot2  seg 4        pickup1 -> shoot2    51.7 in   then SHOOT
 *   pickup2   segs 5 + 6   shoot2  -> pickup2   79.4 in   71.9 deg corner at MID_2
 *   toShoot3  seg 7        pickup2 -> shoot3    64.5 in   then SHOOT
 *   pickup3   seg 8        shoot3  -> pickup3   80.5 in
 *             + seg 9 CUBIC
 *   toShoot4  seg 10       pickup3 -> shoot4    49.5 in   then SHOOT
 *                                              ------- 395.6 in total
 *
 * FAR RANGE IS THE WHOLE DIFFERENCE. All four shots fire from essentially one spot,
 * (59.3-59.8, 21.0-21.5) at heading 115, at a solved range of 125.5-126.2 in. That is
 * within 1.0 in of 32008's own tuned FAR_FIRE_DISTANCE of 126.5 -- closer than the
 * previous revision managed. These shots sit right where their flywheel and hood
 * polynomials were actually fitted, which is the opposite of BlueCloseAuto's situation
 * and a good reason to trust this auto's shot quality more.
 *
 * SPOOL, not settle, is what gates a far shot. The solve asks for ~1878 rpm here
 * against ~1300 on the close side, so the flywheel needs real time to get there and
 * real time to recover after each feed. The drivers measured 1.0-1.8 s. MIN_SPOOL_MS
 * is a hard FLOOR at their lower bound; the measured shooterReady() check carries it
 * the rest of the way, so a shot that needs the full 1.8 s waits for it and one that
 * is already up to speed does not pay for it.
 *
 * SHOT TRIGGER: ARRIVAL, then MEASURED stillness. The leg ending starts the volley, the
 * chassis is parked on Pedro's own position hold (see parkChassis), the rotation
 * estimate is re-seeded, and firing waits on chassis stopped + turret locked + flywheel
 * at speed, held continuously for SETTLE_SHOOT_n. The chassis-stopped requirement is
 * hard: no deadline overrides it. The turret is parked for the feed on BOTH paths into
 * the fire step, and released again the moment readiness is lost.
 *
 * THE WOBBLE WAS THE PARK ITSELF. All four volleys wobbled and all four missed. The
 * cause was not too little settle time -- it was that the old park
 * (startTeleopDrive + zero vector) put the follower in a branch of Follower.update()
 * that applies NO translational correction and DOES apply a centripetal term whose
 * teleop curvature formula divides by averageVelocity.x. At a standstill that is
 * sensor noise, curvature explodes, and the clamp saturates to full sideways power in
 * a sign-flipping direction, every loop. The settle timer was waiting for stillness
 * that the park was actively destroying. parkChassis() now uses holdPoint(), whose
 * branch has real position correction and no centripetal term at all. Full derivation
 * from the 2.1.2 source is on parkChassis().
 *
 * TIMING. The previous revision cut 3.2 s (READY_TIMEOUT_1/2 2.5 -> 1.5 s, and
 * INTAKE_TIMEOUT_1 3.0 -> 1.8 s, measured at the wall). This revision adds the drivers'
 * 0.55 s of sustained-ready hold to all four shots and pays for it in full by moving
 * the same time out of MIN_SPOOL_MS (1000 -> 450 ms), so the pre-fire floor stays at
 * 1.20 s per shot and TOTAL AUTO TIME IS UNCHANGED. The swap trades blind waiting for
 * measured waiting -- see MIN_SPOOL_MS for why that is the stronger of the two.
 *
 * INTAKE RUNS THE WHOLE TIME, moving or stopped, from start() to the last shot.
 *
 * NO PARK LEG. The export ends at shoot 4 and does not drive anywhere afterwards, so
 * the previous version's PARK_POSE/PARK_DEADLINE are gone -- there is nothing to park
 * to. ABORT_DEADLINE stops the mechanisms and holds position instead.
 *
 * FIELD FRAME: the kernel states goals in the PINPOINT frame (pinX = pedroY,
 * pinY = 144 - pedroX). Converted to Pedro below. Blue is (8, 136);
 * (136, 136) is the RED goal.
 */
@Autonomous(name = "32008 Blue Far Auto", group = "32008")
@Configurable
public class BlueFarAuto extends OpMode {

    // Segment 1's own start point. The export's setStartingPose said (72, 8), which is
    // not where its own first path begins -- Pedro would snap hard at launch. Same
    // discrepancy the close export had; the path's own number wins.
    private static final Pose START_POSE    = new Pose(55.553,  7.404, Math.toRadians(90));

    private static final Pose SHOOT_1_POSE  = new Pose(59.339, 21.512, Math.toRadians(115));
    private static final Pose MID_1_POSE    = new Pose(45.214, 35.586, Math.toRadians(180));
    private static final Pose PICKUP_1_POSE = new Pose( 9.692, 35.338, Math.toRadians(180));

    private static final Pose SHOOT_2_POSE  = new Pose(59.450, 21.239, Math.toRadians(115));
    private static final Pose MID_2_POSE    = new Pose(46.703, 59.222, Math.toRadians(180));
    private static final Pose PICKUP_2_POSE = new Pose( 7.419, 58.758, Math.toRadians(180));

    private static final Pose SHOOT_3_POSE  = new Pose(59.721, 21.035, Math.toRadians(115));
    // Seg 8 is now a LINE out to here, turning to -90 on the way. Seg 9 is the cubic
    // that sweeps down to PICKUP_3. This pair replaces the previous export's
    // curve-then-line, which is what put the robot into the wall.
    private static final Pose MID_3_POSE    = new Pose(27.685, 40.224, Math.toRadians(-90));
    private static final Pose PICKUP_3_POSE = new Pose(12.169,  7.766, Math.toRadians(-90));

    private static final Pose SHOOT_4_POSE  = new Pose(59.827, 20.964, Math.toRadians(115));

    /**
     * Seg 9, the one curve on this path -- now a CUBIC, and it fixes the wall bump.
     *
     * WHY THE OLD ONE HIT THE WALL. The previous export's curve ended at (11.454,
     * 44.178) with a minimum radius of 1.12 in, and Pedro's centripetal correction
     * saturates above 12.8 in/s at that radius. The leg ran far faster than that, so
     * the correction ran out of authority and the robot pushed OUTWARD off the arc --
     * outward being straight at the x = 0 wall it was already hugging at x = 11.5.
     * Not a pose error; the geometry demanded more lateral grip than the follower had.
     *
     * WHY THIS ONE DOES NOT. Measured over the new cubic:
     *
     *   minimum radius        8.76 in at t = 0.28   (was 1.12 in)
     *   centripetal saturates above 35.8 in/s       (was 12.8 in/s)
     *   closest approach to the left wall  x = 11.97 in at t = 0.71
     *
     * The margin is now the right way round: the leg needs ~27 in/s average and the
     * geometry holds to 35.8 in/s. The curve also never gets closer to the wall than
     * it starts, so there is no saturating correction pushing it there.
     *
     * The chain is shorter too -- 80.5 in against the old 91.6 in.
     */
    private static final Pose SEG9_C1 = new Pose( 8.089, 42.290);
    private static final Pose SEG9_C2 = new Pose(12.517, 25.203);

    /** Per-leg drive power. maxPower(1) in Constants is already the ceiling. */
    public static double MAX_POWER = 1.0;

    /**
     * Pedro's deceleration knobs, applied per chain so nothing leaks into
     * BlueCloseAuto or the teleop follower. BRAKING_STRENGTH multiplies a MEASURED
     * property of this robot; above 1.0 you assert it stops harder than measured.
     *
     * TODO(UNTUNED): left at the team's current values, so this changes nothing until
     *   somebody measures. Raise BRAKING_STRENGTH in 0.1 steps from Panels, watching
     *   end-of-leg X/Y against the target pose, and stop one step before overshoot.
     */
    public static double BRAKING_STRENGTH = 1.0;
    public static double BRAKING_START = 1.0;

    /** BLUE goal, PEDRO frame. Must read 8 / 136 -- 136 / 136 is the RED goal. */
    public static double BLUE_GOAL_X = 144.0 - RobotConstants.BLUE_TARGET_Y;
    public static double BLUE_GOAL_Y = RobotConstants.BLUE_TARGET_X;

    /** Turret mounting trim, degrees, applied to EVERY shot. Per-shot trim is below. */
    public static double YAW_OFFSET = 0.0;

    /**
     * PER-SHOT AIM TRIM, degrees. Added to YAW_OFFSET for one volley only.
     *
     * SIGN: POSITIVE = LEFT, NEGATIVE = RIGHT. From AutoAimSubsystem:
     *
     *     compensatedTargetAbsAngle = aimResult.algYaw + yawOffset + ...
     *     currentTurretAbsAngle     = smoothHeading + filteredTurretRelAngle
     *
     * Both are field-absolute degrees in Pedro's convention, which is CCW-positive, so
     * adding to yawOffset swings the aim point counter-clockwise. To move the turret
     * RIGHT the trim must be NEGATIVE.
     *
     * SHOT 2 gets -0.5 deg: the drivers asked for a teeny nudge right on that volley.
     * 0.5 deg is about one aim tolerance at this range (see aim.currentTolerance in
     * telemetry) and 1.1 in of lateral movement at the 126 in shot distance -- small
     * enough to be a trim, large enough to actually register.
     *
     * These are DRIVER TRIMS, not measured geometry: they correct where the balls
     * actually land, which folds in ball wear, hood repeatability and turret backlash.
     * That is why they are per-shot and Panels-live rather than a single mounting
     * constant. If ALL four shots start pulling the same way, move it into YAW_OFFSET
     * instead and zero these.
     *
     * Applied from the start of the DRIVE_TO_SHOOT leg, not at the volley, so the
     * turret is already tracking the trimmed angle on arrival.
     */
    public static double YAW_TRIM_1 = 0.0;
    public static double YAW_TRIM_2 = -0.5;
    public static double YAW_TRIM_3 = 0.0;
    public static double YAW_TRIM_4 = 0.0;

    /**
     * PER-SHOT FLYWHEEL TRIM, same units as aim.targetRpm. Added to the solved target
     * for one volley only. POSITIVE = MORE POWER = the ball carries further.
     *
     * SHOTS 1 AND 4 get +25: the drivers asked for a teeny bit more on the preload and
     * on the final volley. For scale, the far solve commands roughly 1878, so +25 is
     * about 1.3%, and VELOCITY_TOR is 20 -- so this is just over one tolerance band.
     * Below about 20 the change would sit inside the readiness deadband and you could
     * not tell whether it had taken effect.
     *
     * ROUTED THROUGH commandedRpm(), so the trim reaches BOTH setShooterVelocity() and
     * shooterReady(). Trimming only the command would leave readiness checking against
     * the untrimmed number, and the shot would fire while the wheel was still climbing
     * the last 25 -- which is the exact failure the trim is meant to fix.
     */
    public static double RPM_TRIM_1 = 25.0;
    public static double RPM_TRIM_2 = 0.0;
    public static double RPM_TRIM_3 = 0.0;
    public static double RPM_TRIM_4 = 25.0;

    /** Gate servo travel. Nothing on this robot senses gate position. */
    public static long GATE_TRAVEL_MS = 400;
    /** Feed window once firing actually starts. */
    public static long TOTAL_SHOOT_TIME_MS = 550;

    /**
     * HARD FLOOR on spool-up before a far shot may fire, milliseconds.
     *
     * The drivers' own measurement: a far shot needs 1.0-1.8 s of flywheel spool or it
     * leaves short. This is set at the LOWER bound of that range on purpose, because it
     * is a floor and not the whole answer -- shooterReady() then holds the shot until
     * the flywheel is actually within VELOCITY_TOR of the ~1878 rpm the solve asked
     * for. A shot needing the full 1.8 s waits for it; one already up to speed fires at
     * 1.0 s instead of paying a worst-case tax on every volley.
     *
     * Why far needs this and close does not: the far solve commands ~1878 rpm against
     * ~1300 close, so both the initial spin-up and the post-feed recovery take longer,
     * and the preload leg here is only 16.2 in -- barely any approach to spool during.
     *
     * CUT 1000 -> 450 TO PAY FOR THE 0.55 s SETTLE, at equal wall-clock. This is a
     * straight swap of BLIND waiting for MEASURED waiting, and it is the only place
     * 0.55 s per shot could come from without lengthening the auto:
     *
     *   before   1.000 s blind floor  + 0.20 s sustained-ready  = 1.20 s pre-fire
     *   after    0.450 s blind floor  + 0.75 s sustained-ready  = 1.20 s pre-fire
     *
     * The 0.55 s moved from this floor into SETTLE_SHOOT_n is not lost spool time. The
     * flywheel is commanded to aim.targetRpm every loop regardless of state, so it
     * keeps spooling right through the settle window; and settle only counts while
     * readyToFire() is CONTINUOUSLY true, which includes shooter.shooterReady(). A
     * flywheel that dips below tolerance mid-settle resets the settle timer to zero.
     * So the replacement demands 0.75 s of proven at-speed running where the floor
     * demanded 0.55 s of unexamined waiting -- strictly the stronger check.
     *
     * 450 still covers GATE_TRAVEL_MS (400), which is the other thing the phase-1 hold
     * is protecting.
     *
     * If far shots start landing short after this, put this back to 1000 and accept the
     * +2.2 s rather than widening VELOCITY_TOR; a wider tolerance would let an
     * under-speed flywheel report ready and would defeat the settle window too.
     */
    public static long MIN_SPOOL_MS = 450;

    /**
     * PER-SHOT give-up deadline: fire anyway after this long waiting on locked +
     * at-speed. Measured from the start of the volley, so MIN_SPOOL_MS eats the first
     * 1.0 s of it.
     *
     * This is the ONLY knob that was cut to save the drivers' requested time, because
     * it is the only one whose cost is pure waiting. Everything that makes the shot
     * stable is untouched: MIN_SPOOL_MS still holds the full 1.0 s spool floor,
     * SETTLE_SHOOT_n still holds 0.2 s after lock, and chassisStopped() is still a
     * hard requirement that no timeout can override.
     *
     * Shots 1 and 2 are cut 2.5 -> 1.5 s (exactly 1.0 s each, drivers' request), so the
     * robot leaves for the next leg a second sooner when a volley is dragging. When a
     * volley is already ready on time, these change nothing -- the settled path fires
     * first and the deadline never comes up.
     *
     * Shots 3 and 4 keep 2.5 s. They were not asked about, and shot 4 is the volley
     * with the longest rotation into it (155 deg over 49.5 in).
     */
    public static double READY_TIMEOUT_1 = 1.5;
    public static double READY_TIMEOUT_2 = 1.5;
    public static double READY_TIMEOUT_3 = 2.5;
    public static double READY_TIMEOUT_4 = 2.5;

    /**
     * SUSTAINED-READY hold before firing, per shot: 0.20 -> 0.75 s, the drivers' +0.55 s
     * on every volley after all four wobbled.
     *
     * This is not a plain sleep. The clock starts only once readyToFire() is already
     * true -- chassis measured still, turret locked, flywheel at the solved rpm -- and
     * it RESTARTS FROM ZERO the instant any of those drops. So 0.75 s here means 0.75
     * seconds of continuously proven stillness, not 0.75 seconds of hoping.
     *
     * Paid for by MIN_SPOOL_MS 1000 -> 450, so the pre-fire floor stays at 1.20 s per
     * shot and the auto does not get longer. See MIN_SPOOL_MS.
     *
     * NOW PER-SHOT, on what the drivers actually saw:
     *
     *   SHOOT_1  0.25 -> 0.35   +0.10 s.  Preload chassis motion still visible, one ball
     *            missed. Was cut to 0.25 when the preload looked clean; it is not.
     *   SHOOT_2  0.90           unchanged, held from the drivers' earlier +0.15.
     *   SHOOT_3  0.75 -> 0.89   +0.14 s.  The second-row round also wobbled.
     *   SHOOT_4  0.75           unchanged. The final round wobbled last time but no
     *            extra time was asked for, and the cause is addressed structurally by
     *            the re-anchor in shotComplete() phase 1. If it still moves, THIS is
     *            the knob -- raise it, it is Panels-live.
     */
    public static double SETTLE_SHOOT_1 = 0.35;
    public static double SETTLE_SHOOT_2 = 0.90;
    public static double SETTLE_SHOOT_3 = 0.89;
    public static double SETTLE_SHOOT_4 = 0.75;

    /**
     * WHAT "STOPPED" MEANS, measured off the follower every loop. Same rationale as
     * BlueCloseAuto: pathDone() fires at t > 0.99 with the robot still moving, and
     * cutting power lets it coast rather than stopping it.
     *
     * TIGHTENED with the 0.75 s settle, because the two multiply. The old 2.0 in/s and
     * 5 deg/s, sustained across a 0.75 s window, permit 1.5 in of drift and 3.75 deg of
     * rotation while the shot is supposedly "still" -- and at this auto's 126 in range,
     * 1.5 in of lateral drift is 0.7 deg of pointing error on its own. That is the size
     * of a miss. 1.0 in/s and 2.0 deg/s cut the worst-case drift to 0.75 in / 1.5 deg.
     *
     * Now reachable where it was not before: parkChassis() actively corrects position
     * instead of leaving the chassis with no feedback and a saturating centripetal kick.
     *
     * TODO(UNTUNED): detection thresholds, not measured robot properties. 1.0 in/s is
     *   Pedro's own stuckVelocity default, the nearest published reference.
     *
     * RELAX THESE FIRST if telemetry starts reading "shot: ABANDONED on timeout" -- that
     *   means the chassis never met the threshold inside SHOOT_TIMEOUT and the ball was
     *   given up, which is worse than a slightly loose shot. Both are Panels-live.
     */
    public static double STOPPED_SPEED_MAX = 1.0;
    public static double STOPPED_TURN_MAX = 2.0;

    /**
     * Park the turret once everything reports ready, so it cannot hunt through the
     * shot. Their turret feedforward applies static friction as a bang-bang term and
     * limit-cycles otherwise -- see AutoAimSubsystem.holdTurret.
     */
    public static boolean FREEZE_TURRET_ON_LOCK = true;

    // Safety rails. 395.6 in of path plus four far volleys budgets well inside 30 s.
    public static double PATH_TIMEOUT = 7.0;

    /**
     * How long Pedro's stuck flag must hold before a drive-to-shoot leg gives up on it.
     * Matches the library's own stuckTimeout (500 ms), which is the delay this file
     * used to skip by reading isRobotStuck() directly. See pathDone(double, boolean).
     */
    public static double STUCK_CONFIRM_S = 0.5;

    /**
     * Per-leg collection time, seconds. Every pickup pose sits hard against a wall
     * (x = 9.7, 9.3, and 11.3 at y = 8.4), so as on the close auto these legs never
     * report parametric end -- they push until the timeout expires, and the timeout IS
     * the leg duration.
     *
     * LEG 1 IS NOW DRIVER-MEASURED: cut 3.0 -> 1.8 s, exactly the 1.2 s they watched the
     * robot spend parked against the wall with nothing left to collect. That is the
     * "reaches the wall early and then sits there" case the note below predicted.
     *
     * FLAGGED: 55.5 in in 1.8 s is a 30.8 in/s average, and this chain is not a straight
     * run -- seg 2 turns 65 deg over its 19.9 in before seg 3's 35.5 in wall approach.
     * The measured drivetrain (81 in/s forward, 64 lateral) covers it, but there is much
     * less slack than the old 18.5 in/s. If the robot is still short of the wall when
     * this expires, the balls are left on the field -- raise it back in 0.2 s steps.
     *
     * LEG 2, 3.0 -> 3.3 s. It was being CANCELLED, not timed out -- see
     * pathDone(double, boolean) for the stuck-detector bug that ended it after two
     * balls. With that fixed the leg gets its whole window back, and this export also
     * pushed PICKUP_2 1.92 in deeper into the wall (x 9.292 -> 7.419), taking the chain
     * to 79.35 in. 3.3 s puts the required average at 24.0 in/s against the ~30.8 in/s
     * leg 1 actually measured, so there is real dwell at the wall for the third ball
     * instead of arriving just as the timer expires.
     *
     * TODO(UNTUNED): legs 2 and 3 still are not stopwatched. Leg 3 carries 3.0 s at
     *   80.5 in (26.8 in/s). Leg 3 used to be 91.6 in with a 1.12 in-radius corner in
     *   it, and those together
     *   were the numbers on the export that drove the robot into the wall. The new
     *   seg 8 + seg 9 pair shortens leg 3 to 80.5 in (26.8 in/s at 3.0 s) and its
     *   curve now holds to 35.8 in/s before centripetal saturates, so this leg finally
     *   has margin instead of a deficit. Same rule as leg 1 -- if it reaches the wall
     *   early and sits, cut it; if it is still short when the timer expires, raise it.
     */
    public static double INTAKE_TIMEOUT_1 = 1.8;
    public static double INTAKE_TIMEOUT_2 = 3.3;
    public static double INTAKE_TIMEOUT_3 = 3.0;

    /** Room for MIN_SPOOL + READY_TIMEOUT + the feed window, with margin. */
    public static double SHOOT_TIMEOUT = 5.0;
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
    /** Runs from the moment everything reports ready, not from arrival. */
    private final Timer settleTimer = new Timer();
    private boolean wasReady = false;
    private boolean turretFrozen = false;
    /** One re-anchor per volley, at the first loop the chassis actually reads still. */
    private boolean anchored = false;
    /** Leg-relative time stuck was first seen, or negative if not seen this leg. */
    private double stuckSince = -1.0;

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
        // aass = true: AutoAim owns turret "lt" AND hood "panel". Without this both
        // classes grab the turret and fight over its run mode.
        shooter.init(hardwareMap, true);
        // Zero the turret HERE and only here, with it parked forward. Put the flag
        // straight back so TeleOp inherits this zero instead of re-zeroing.
        AutoAimSubsystem.RESET_TURRET_ENCODER_ON_INIT = true;
        autoAim.init(hardwareMap);
        AutoAimSubsystem.RESET_TURRET_ENCODER_ON_INIT = false;

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

    /** Every control point and heading interpolation is the export's, verbatim. */
    private void buildPaths() {
        toShoot1 = brake(follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115)))
                .build();

        // EXPORT SLIP, deliberately NOT copied: this segment's export reads
        // setLinearHeadingInterpolation(toRadians(0), toRadians(180)). The robot is at
        // 115 when this leg starts, so a 0 start heading commands an instant 115 deg
        // snap the wrong way before winding back up to 180. Every other segment
        // boundary in the same export chains correctly (115->180, 180->115, 115->-90,
        // -90->115); only this one field is off, which is what a stale visualizer field
        // looks like. Kept at 115. If 0 really was intended, change it back here.
        pickup1 = brake(follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_1_POSE, MID_1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .addPath(new BezierLine(MID_1_POSE, PICKUP_1_POSE))
                .setTangentHeadingInterpolation())
                .build();

        toShoot2 = brake(follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_1_POSE, SHOOT_2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115)))
                .build();

        pickup2 = brake(follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_2_POSE, MID_2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .addPath(new BezierLine(MID_2_POSE, PICKUP_2_POSE))
                .setTangentHeadingInterpolation())
                .build();

        toShoot3 = brake(follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_2_POSE, SHOOT_3_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115)))
                .build();

        // Seg 8 line out to MID_3 turning to -90, then seg 9's cubic down to the corner.
        //
        // SECOND EXPORT SLIP, deliberately NOT copied: seg 9's export reads
        // setTangentHeadingInterpolation(). The cubic's tangent at t = 0 is 174 deg
        // (it leaves MID_3 heading -X before curving down), but seg 8 has just finished
        // placing the robot at -90. Tangent interpolation there commands a 96 deg snap
        // at the junction and then winds 265 deg back around over the curve. The curve's
        // OWN exit tangent is -91.1 deg, so -90 is what both ends actually want, and a
        // constant -90 is also what the intake needs: this leg drives into the bottom
        // wall face-first, exactly like pickups 1 and 2 drive into the left wall at 180.
        pickup3 = brake(follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_3_POSE, MID_3_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(-90))
                .addPath(new BezierCurve(MID_3_POSE, SEG9_C1, SEG9_C2, PICKUP_3_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90)))
                .build();

        toShoot4 = brake(follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_3_POSE, SHOOT_4_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(115)))
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

        // Aim + shooter + intake run EVERY loop, unconditionally.
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

        // ALLIANCE HANDOFF. TeleOp never picks a colour -- it aims at whatever these
        // hold. Stated in the PINPOINT frame, which is TeleOp's frame, so no conversion.
        RobotConstants.teleOpTargetX = RobotConstants.BLUE_TARGET_X;
        RobotConstants.teleOpTargetY = RobotConstants.BLUE_TARGET_Y;
    }

    private void updateAim() {
        if (turretFrozen) {
            // Motor off; filters, last solve and hood position left exactly as they
            // were so aiming resumes cleanly next state.
            autoAim.holdTurret();
            return;
        }
        Pose p = follower.getPose();
        Vector v = follower.getVelocity();
        double headingDeg = Math.toDegrees(p.getHeading());
        // The shooter does not sit over the centre of rotation.
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
                YAW_OFFSET + yawTrim());
    }

    /**
     * Commands the flywheel every loop. Falls back to shooterHold() rather than to
     * zero when the solve has no target, so a momentary bad solve cannot spin the
     * shooter down -- which at far range would cost far more than it does up close.
     */
    private void driveShooter() {
        if (!shooterLive) {
            return;
        }
        if (aim.hasTarget && aim.targetRpm > 0.0) {
            // Trimmed, and gated on the UNTRIMMED solve above so a trim can never make
            // a no-target loop look like a live one.
            shooter.setShooterVelocity(commandedRpm());
        } else {
            shooter.shooterHold();
        }
    }

    /** Intake runs continuously, moving or stopped. The fire step overrides its power. */
    private void driveIntake() {
        if (intakeLive) {
            intake.intakeIn();
        }
    }

    /**
     * Which volley the robot is working toward, 1-4, or 0 on a pickup leg.
     *
     * Deliberately covers the DRIVE_TO_SHOOT_n leg as well as SHOOT_n. The trims have
     * to be live during the approach so the turret is already tracking the trimmed
     * angle and the flywheel is already at the trimmed speed when the volley starts --
     * applying them at the volley would make every shot wait out a correction.
     */
    private int shotIndex() {
        switch (state) {
            case DRIVE_TO_SHOOT_1: case SHOOT_1: return 1;
            case DRIVE_TO_SHOOT_2: case SHOOT_2: return 2;
            case DRIVE_TO_SHOOT_3: case SHOOT_3: return 3;
            case DRIVE_TO_SHOOT_4: case SHOOT_4: return 4;
            default: return 0;
        }
    }

    /** Per-shot aim trim in degrees; 0 outside a shot approach. See YAW_TRIM_1. */
    private double yawTrim() {
        switch (shotIndex()) {
            case 1:  return YAW_TRIM_1;
            case 2:  return YAW_TRIM_2;
            case 3:  return YAW_TRIM_3;
            case 4:  return YAW_TRIM_4;
            default: return 0.0;
        }
    }

    /**
     * The flywheel speed this loop actually wants: the solve plus the per-shot trim.
     * SINGLE SOURCE for both the command and the readiness check -- see RPM_TRIM_1.
     */
    private double commandedRpm() {
        double trim;
        switch (shotIndex()) {
            case 1:  trim = RPM_TRIM_1; break;
            case 2:  trim = RPM_TRIM_2; break;
            case 3:  trim = RPM_TRIM_3; break;
            case 4:  trim = RPM_TRIM_4; break;
            default: trim = 0.0; break;
        }
        return aim.targetRpm + trim;
    }

    /** Telemetry only -- nothing gates on it. */
    private void trackShootPoint(Pose target) {
        Pose p = follower.getPose();
        distanceToShootPoint = Math.hypot(target.getX() - p.getX(), target.getY() - p.getY());
    }

    /** Drive-to-shoot legs: honour stuck detection, but only once confirmed. */
    private boolean pathDone() {
        return pathDone(PATH_TIMEOUT, true);
    }

    /** Intake legs: the timeout IS the leg duration. Never exit on stuck -- see below. */
    private boolean pathDone(double timeout) {
        return pathDone(timeout, false);
    }

    /**
     * @param allowStuckExit whether Pedro's stuck detector may end this leg early.
     *
     * WHY THIS IS NOW A CHOICE. follower.isRobotStuck() is a one-liner:
     *
     *     public boolean isRobotStuck() { return zeroVelocityDetectedTimer != null; }
     *
     * and Follower.update() creates that timer after a SINGLE loop under
     * stuckVelocity (1.0 in/s) anywhere in t = 0.1 .. 0.8 of the current path.
     * Pedro itself then waits stuckTimeout (500 ms) before acting on it. Reading the
     * flag directly, as this method used to, is therefore 500 ms MORE trigger-happy
     * than the library that owns it -- one noisy loop below 1 in/s and the leg is over.
     *
     * That is what ended pickup 2 early. Its chain turns 71.9 deg at MID_2 (seg 5 runs
     * at 108.6 deg, seg 6 at -179.3 deg) -- by far the sharpest corner on this path;
     * pickup 1 turns 45.3 deg and pickup 3 turns 24.9 deg. The robot has to brake hard
     * through a corner that sharp, dips under 1 in/s just as seg 6's t clears 0.1, the
     * flag latches, and this method ended the leg on the spot. Two balls collected,
     * then an immediate followPath(toShoot3) -- which is exactly the "sprang away, speed
     * so uneven, not like the others" the drivers described. Nothing was wrong with the
     * path or the timeout; the leg was being cancelled.
     *
     * INTAKE LEGS NOW PASS false. Slowing to a crawl is what these legs are FOR -- they
     * brake through a corner and then press a wall on purpose. Stuck detection exists to
     * catch a robot wedged where it should be moving, and that describes none of them.
     * The timeout is the leg duration, as documented on INTAKE_TIMEOUT_n, and
     * ABORT_DEADLINE remains the backstop for a genuinely trapped robot.
     *
     * DRIVE-TO-SHOOT LEGS STILL PASS true, but must now hold the condition for
     * STUCK_CONFIRM_S first, which restores the library's own 500 ms semantics.
     */
    private boolean pathDone(double timeout, boolean allowStuckExit) {
        if (allowStuckExit && follower.isRobotStuck()) {
            // The flag latches until breakFollowing(), so first sighting is enough to
            // start the clock; it will not un-stick underneath us.
            if (stuckSince < 0.0) {
                stuckSince = stateTimer.getElapsedTimeSeconds();
            }
            if (stateTimer.getElapsedTimeSeconds() - stuckSince >= STUCK_CONFIRM_S) {
                lastTransition = "path ended: ROBOT STUCK (confirmed " + STUCK_CONFIRM_S + "s)";
                return true;
            }
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

    /** MEASURED stillness, not assumed. */
    private boolean chassisStopped() {
        return follower.getVelocity().getMagnitude() <= STOPPED_SPEED_MAX
                && Math.abs(Math.toDegrees(follower.getAngularVelocity())) <= STOPPED_TURN_MAX;
    }

    /** Chassis stopped AND turret on target AND flywheel actually at the solved rpm. */
    private boolean readyToFire() {
        return chassisStopped()
                && aim.hasTarget && aim.isAimLocked && shooter.shooterReady(commandedRpm());
    }

    /**
     * Parks the drivetrain on Pedro's own position hold, at the pose the robot is
     * actually standing on right now.
     *
     * THIS REPLACES THE OLD startTeleopDrive + zero-vector PARK, WHICH WAS ITSELF THE
     * WOBBLE. Read Follower.update() (core 2.1.2): the manualDrive branch runs
     *
     *     drivetrain.runDrive(getCentripetalForceCorrection(),
     *                         getTeleopHeadingVector(),
     *                         getTeleopDriveVector(), ...)
     *
     * with NO translational correction at all -- so a zero teleop vector gives the
     * chassis no position feedback whatsoever -- and it still applies centripetal.
     * VectorCalculator.getCentripetalForceCorrection() in teleop mode computes
     *
     *     yPrime       = averageVelocity.y / averageVelocity.x
     *     yDoublePrime = averageAcceleration.y / averageVelocity.x
     *     curvature    = yDoublePrime / (sqrt(1 + yPrime^2))^3
     *
     * At a standstill averageVelocity.x is sensor noise near zero. Exact 0/0 gives NaN
     * and is guarded; a TINY NONZERO x is not, and it makes curvature explode. That
     * feeds
     *
     *     clamp(centripetalScaling * mass * v_tangential^2 * curvature, +/-maxPower)
     *
     * which saturates to FULL SIDEWAYS POWER, in a direction taken from the stale
     * previous path's tangent, with the sign flipping as noise flips the acceleration
     * estimate. A parked robot in that branch gets kicked left-right every loop. That
     * is the wobble, and it is why more settle time alone never fixed it -- the settle
     * timer was waiting for a condition the park was actively preventing.
     *
     * The holdingPosition branch of the same update() runs
     *
     *     drivetrain.runDrive(getTranslationalCorrection() * holdPointTranslationalScaling,
     *                         getHeadingVector() * holdPointHeadingScaling,
     *                         new Vector(), ...)
     *
     * -- real translational and heading correction, softened by Pedro's own 0.45/0.35
     * hold scalings, and NO centripetal term at all. It is the mode Pedro itself drops
     * into at the end of a followPath(holdEnd = true), so this is the library's
     * intended parked state, not a trick.
     *
     * Holding the CURRENT pose rather than the shoot pose is deliberate: it asks the
     * drivetrain for the least motion possible. AutoAim solves from live pose every
     * loop, so a couple of inches of arrival error costs nothing in aim -- driving to
     * remove it would cost motion, which is the thing being eliminated.
     *
     * Call this ONCE per volley. holdPoint() re-snapshots its target and re-runs
     * breakFollowing() on every call, so calling it each loop would drag the hold
     * target along behind the robot.
     */
    private void parkChassis() {
        follower.holdPoint(follower.getPose());
    }

    /**
     * One far volley. Differs from the close auto in two places: the phase-1 hold is
     * max(GATE_TRAVEL_MS, MIN_SPOOL_MS) rather than gate travel alone, so a far shot
     * cannot fire before the flywheel has had the drivers' measured minimum spool time
     * no matter what shooterReady() claims; and the READY_TIMEOUT bail-out is itself
     * gated on chassisStopped(), so no path through this method feeds the intake while
     * the robot is still moving.
     *
     * @param settle       extra parked-and-locked hold once readyToFire() is true
     * @param readyTimeout per-shot give-up deadline, measured from volley start
     */
    private boolean shotComplete(double settle, double readyTimeout) {
        // No per-loop re-assert here any more. parkChassis() installs Pedro's
        // holdingPosition mode once and follower.update() maintains it for the rest of
        // the volley; nothing in this method clears it. The old re-assert existed to
        // keep re-sending a zero teleop vector, and that whole park is gone.
        if (stateTimer.getElapsedTimeSeconds() > SHOOT_TIMEOUT) {
            intake.gateClose();
            shotPhase = 0;
            shotsFired++;
            lastTransition = "shot: ABANDONED on timeout";
            return true;
        }

        switch (shotPhase) {

            case 0:
                // Stop the chassis for real. pathDone() can fire on atParametricEnd()
                // while the follower is still busy running the full path PIDF, so the
                // hold has to be installed explicitly rather than waited for.
                parkChassis();
                // Drop the stale rotation estimate so the turret stops counter-rotating
                // against a spin that has ended. Matters most on toShoot4, which turns
                // 155 deg over 50.1 in -- 2x the rotation rate of any other leg here.
                autoAim.resetHeadingFilter();
                intake.gateOpen();
                intake.intakeEngage();
                shotTimer.resetTimer();
                settleTimer.resetTimer();
                wasReady = false;
                turretFrozen = false;
                anchored = false;
                shotPhase = 1;
                lastTransition = "shot: chassis held, gate opening, spooling";
                return false;

            case 1: {
                // RE-ANCHOR AT TRUE REST, once per volley. Both of the shots the drivers
                // called out as visibly wobbling -- shot 2 and shot 4 -- are the ones
                // that arrive carrying the most momentum, and both wobbles come from the
                // same thing: phase 0 has to fire while the robot is still moving,
                // because pathDone() triggers at atParametricEnd() and the path PIDF has
                // to be killed right then. So both the hold point and the rotation
                // filter get captured mid-coast.
                //
                //   CHASSIS: holdPoint() latched the pose the robot was passing through,
                //     not the one it stops at. It then drags the robot back that far --
                //     visible motion during the volley, worst on the legs with the most
                //     momentum. Re-anchoring at rest gives the hold zero standing error,
                //     so it has nothing to pull against.
                //
                //   TURRET: resetHeadingFilter() re-seeded the rotation estimate off a
                //     robot that was still turning, so the filter starts already holding
                //     a rotation rate that no longer exists, and AutoAim feeds that
                //     straight into the turret as -filteredRobotOmega. Worst on
                //     toShoot4, which turns 155 deg over 49.5 in -- by far the highest
                //     rotation rate of any leg. Re-seeding from a stationary robot
                //     starts it at zero.
                //
                // Checked BEFORE the spool hold below, so it fires the instant the robot
                // is actually still rather than waiting on the flywheel.
                if (!anchored && chassisStopped()) {
                    anchored = true;
                    parkChassis();
                    autoAim.resetHeadingFilter();
                    lastTransition = "shot: re-anchored at rest (chassis + heading filter)";
                }

                // Gate travel AND the far-shot spool floor, whichever is longer.
                long hold = Math.max(GATE_TRAVEL_MS, MIN_SPOOL_MS);
                if (shotTimer.getElapsedTime() < hold) {
                    return false;
                }
                boolean ready = readyToFire();
                if (ready && !wasReady) {
                    wasReady = true;
                    settleTimer.resetTimer();
                    aimErrorAtFreeze = aim.aimError;
                    if (FREEZE_TURRET_ON_LOCK) turretFrozen = true;
                    lastTransition = "shot: READY (stopped+locked+at speed), settling " + settle + "s";
                } else if (!ready && wasReady) {
                    wasReady = false;
                    // HAND THE TURRET BACK. This is the missed-ball bug, not just an
                    // optimisation. updateAim() early-returns while turretFrozen is
                    // set, so the whole `aim` struct stops updating -- target angle,
                    // lock flag, rpm, all frozen at the values from the instant of
                    // lock. If the chassis then moved, the turret is now holding a
                    // solution for a pose the robot has left, and readyToFire() re-reads
                    // that same stale lock the moment the chassis settles again: it
                    // re-freezes, re-settles, and fires at the OLD aim point. Clearing
                    // the freeze here forces a live re-solve from the pose the robot
                    // actually ended up at before the shot can arm again.
                    turretFrozen = false;
                    lastTransition = "shot: readiness LOST, turret released, settle restarts";
                }

                boolean settled = wasReady && settleTimer.getElapsedTimeSeconds() >= settle;
                // The bail-out gives up on the TURRET and the FLYWHEEL, never on the
                // chassis. Feeding a ball while the robot is still drifting throws it
                // wherever the drift points, so stillness is the one condition with no
                // timeout override -- SHOOT_TIMEOUT above is the only escape, and it
                // abandons the shot rather than taking a bad one.
                boolean gaveUp = shotTimer.getElapsedTimeSeconds() > readyTimeout + settle
                        && chassisStopped();
                if (settled || gaveUp) {
                    // Park the turret for the feed itself, on BOTH paths into phase 2.
                    // The settled path already froze it at lock; the give-up path did
                    // not, and an unlocked turret is still actively hunting -- their
                    // feedforward bang-bangs static friction and limit-cycles, which is
                    // exactly the left-right wobble to keep out of a shot. Nothing
                    // reads the turret again until setState() hands it back.
                    if (FREEZE_TURRET_ON_LOCK) turretFrozen = true;
                    intake.intakeFire(shooter.calculateIntakePower());
                    shotTimer.resetTimer();
                    shotPhase = 2;
                    lastTransition = settled
                            ? "shot: FIRING (spooled + locked + settled)"
                            : "shot: FIRING (deadline " + readyTimeout + "s, stopped -- may be under speed)";
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
                if (shotComplete(SETTLE_SHOOT_1, READY_TIMEOUT_1)) {
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
                if (shotComplete(SETTLE_SHOOT_2, READY_TIMEOUT_2)) {
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
                if (shotComplete(SETTLE_SHOOT_3, READY_TIMEOUT_3)) {
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
                if (shotComplete(SETTLE_SHOOT_4, READY_TIMEOUT_4)) {
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
     * mechanisms and holds wherever the robot is.
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
        stuckSince = -1.0;
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
        panelsTelemetry.debug("Speed at shoot pt", follower.getVelocity().getMagnitude());
        panelsTelemetry.debug("Turn rate (deg/s)", Math.toDegrees(follower.getAngularVelocity()));
        panelsTelemetry.debug("Dist to shoot pt", distanceToShootPoint);
        panelsTelemetry.debug("Follower busy", follower.isBusy());
        panelsTelemetry.debug("Robot stuck", follower.isRobotStuck());

        panelsTelemetry.debug("Ready to FIRE", readyToFire());
        panelsTelemetry.debug("Chassis STOPPED", chassisStopped());
        panelsTelemetry.debug("Turret FROZEN", turretFrozen);
        panelsTelemetry.debug("Aim error at freeze", aimErrorAtFreeze);
        panelsTelemetry.debug("Flywheel at speed", shooter.shooterReady(commandedRpm()));
        panelsTelemetry.debug("Shot index", shotIndex());
        panelsTelemetry.debug("Yaw trim (deg)", yawTrim());
        panelsTelemetry.debug("RPM trim", commandedRpm() - aim.targetRpm);
        panelsTelemetry.debug("Intake live", intakeLive);
        panelsTelemetry.debug("Min spool (ms)", MIN_SPOOL_MS);
        panelsTelemetry.debug("Aim has target", aim.hasTarget);
        panelsTelemetry.debug("Aim LOCKED", aim.isAimLocked);
        panelsTelemetry.debug("Aim range (in)", aim.targetDist);
        panelsTelemetry.debug("Aim error (deg)", aim.aimError);
        panelsTelemetry.debug("Aim tolerance (deg)", aim.currentTolerance);
        panelsTelemetry.debug("Turret target (deg)", aim.targetTurretAngle);
        panelsTelemetry.debug("Turret actual (deg)", autoAim.getCurrentTurretAngle());

        panelsTelemetry.debug("HOOD cmd (percent)", aim.targetPitch);
        panelsTelemetry.debug("HOOD servo pos", autoAim.hood.getPosition());
        panelsTelemetry.debug("Shooter target (solve)", aim.targetRpm);
        panelsTelemetry.debug("Shooter target (commanded)", commandedRpm());
        panelsTelemetry.debug("Shooter actual", shooter.getShooterVelocity());
        panelsTelemetry.debug("Intake fire power", shooter.calculateIntakePower());
        panelsTelemetry.debug("Battery (V)", autoAim.getCurrentBatteryVoltage());
        panelsTelemetry.update(telemetry);
    }
}
