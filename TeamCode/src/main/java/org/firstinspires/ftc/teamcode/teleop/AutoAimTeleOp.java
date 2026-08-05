package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.kernel.constants.robotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoAimSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Team 32008 -- DECODE 2025-26 -- driver-controlled period, both alliances.
 *
 * Same architecture as the auto: 32008's own {@link AutoAimSubsystem},
 * {@link Shooter} and {@link Intake}, copied verbatim, with the turret solving
 * from the live pose every loop. Control layout follows their competition
 * teleop (V2 tests/A_1_AA_AS.java and tests/AASSTEST.java).
 *
 * LOCALIZATION: Pedro's follower, not a raw Pinpoint. 32008's teleop drives the
 * Pinpoint directly and converts frames at every call site; here the auto and the
 * teleop share one follower configuration, so the pose that comes out of auto
 * goes straight in with no conversion and no chance of getting the transform
 * backwards.
 *
 * POSE HANDOFF: seeded from kernel robotConstants.autoEndX/Y/H, which the auto
 * writes every loop. Those are static and survive between OpModes -- so if you
 * run TeleOp WITHOUT running auto first, they hold whatever the last run left.
 * The init screen shows the seeded pose; OPTIONS re-seeds to a known corner.
 *
 * MECHANISMS ARE NOT GATED ON THE AIM SOLVE, same rule as the auto. Drive,
 * intake and gate work whether or not the turret has a solution.
 *
 * CONTROLS -- gamepad 1
 *   left stick            drive (field centric by default)
 *   right stick X         turn
 *   left bumper           TOGGLE aim + shooter
 *   right bumper (hold)   FIRE -- feeds at the distance-scaled intake power
 *   right trigger         intake in  (cancels aim mode)
 *   left trigger          intake out, proportional to the trigger
 *   A                     hold for slow mode
 *   B                     toggle field centric / robot centric
 *   OPTIONS               re-seed pose to RESEED_POSE (localization recovery)
 *   dpad up / down        shot distance trim  +/- 2 in
 *   dpad left / right     turret trim         +/- 1 deg
 *
 * CONTROLS -- gamepad 2 (same trims, plus)
 *   left bumper           toggle auto turret / manual (turret parks straight)
 */
@Configurable
public abstract class AutoAimTeleOp extends OpMode {

    /** Goal for this alliance, PEDRO frame. */
    protected abstract double goalX();
    protected abstract double goalY();
    protected abstract String allianceName();

    /** Where OPTIONS re-seeds the robot to when localization is lost. */
    public static Pose RESEED_POSE = new Pose(56.0, 8.0, Math.toRadians(90));

    public static double DRIVE_SCALE = 1.0;
    public static double SLOW_SCALE = 0.35;
    public static double TURN_SCALE = 0.9;
    public static double STICK_DEADBAND = 0.05;

    /** Hood/flywheel park range while the shooter is idling, inches. */
    public static double HOLD_DISTANCE = 126.5;

    /** dpad trim step sizes, matching their teleop. */
    public static double DISTANCE_TRIM_STEP = 2.0;
    public static double TURRET_TRIM_STEP = 1.0;

    /** Manual-mode fallback range when the turret is taken off auto. */
    public static double MANUAL_DISTANCE = 126.5;

    private Follower follower;
    private TelemetryManager panelsTelemetry;

    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final AutoAimSubsystem autoAim = new AutoAimSubsystem();

    private AutoAimSubsystem.TurretCommand aim = new AutoAimSubsystem.TurretCommand();

    private boolean aimOn = false;
    private boolean autoTurret = true;
    private boolean fieldCentric = true;
    private boolean firing = false;

    private double distanceTrim = 0.0;
    private double turretTrim = 0.0;

    private boolean prevLeftBumper1 = false, prevB1 = false, prevOptions1 = false;
    private boolean prevLeftBumper2 = false;
    private boolean prevDpadUp = false, prevDpadDown = false;
    private boolean prevDpadLeft = false, prevDpadRight = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(
                robotConstants.autoEndX,
                robotConstants.autoEndY,
                robotConstants.autoEndH));

        intake.init(hardwareMap);
        // aass = true: AutoAim owns turret "lt" AND hood "panel". Without this
        // both classes grab the turret and fight over its run mode.
        shooter.init(hardwareMap, true);
        autoAim.init(hardwareMap);

        intake.gateClose();

        panelsTelemetry.debug("Status", "Initialized -- " + allianceName());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        follower.update();
        panelsTelemetry.debug("Alliance", allianceName());
        panelsTelemetry.debug("Goal X (Pedro)", goalX());
        panelsTelemetry.debug("Goal Y (Pedro)", goalY());
        // Verify this against where the robot actually is. If auto did not run,
        // these are stale -- press OPTIONS after START to re-seed.
        panelsTelemetry.debug("Seeded X (from auto)", follower.getPose().getX());
        panelsTelemetry.debug("Seeded Y (from auto)", follower.getPose().getY());
        panelsTelemetry.debug("Seeded heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Turret ticks", autoAim.getCurrentTick());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();

        handleDrive();
        handleTrims();
        handleToggles();
        // Before updateAim/handleShooterAndGate: pulling the intake trigger cancels
        // aim mode, and the gate must see that in the SAME loop or it stays open
        // for a cycle while the intake is already running into it.
        handleIntake();

        // Aim runs EVERY loop regardless of mode, so the turret filters stay warm
        // and the turret is already tracking the instant aim is switched on.
        updateAim();
        handleShooterAndGate();

        report();
    }

    @Override
    public void stop() {
        shooter.shooterStop();
        intake.intakeStop();
        intake.gateClose();
        autoAim.stop();
    }

    private double deadband(double v) {
        return Math.abs(v) < STICK_DEADBAND ? 0.0 : v;
    }

    private void handleDrive() {
        double scale = gamepad1.a ? SLOW_SCALE : DRIVE_SCALE;
        double forward = deadband(-gamepad1.left_stick_y) * scale;
        double strafe  = deadband(-gamepad1.left_stick_x) * scale;
        double turn    = deadband(-gamepad1.right_stick_x) * TURN_SCALE * scale;

        // Pedro's isRobotCentric flag is the inverse of field centric.
        follower.setTeleOpDrive(forward, strafe, turn, !fieldCentric);
    }

    private void handleTrims() {
        boolean up    = gamepad1.dpad_up    || gamepad2.dpad_up;
        boolean down  = gamepad1.dpad_down  || gamepad2.dpad_down;
        boolean left  = gamepad1.dpad_left  || gamepad2.dpad_left;
        boolean right = gamepad1.dpad_right || gamepad2.dpad_right;

        if (up    && !prevDpadUp)    distanceTrim += DISTANCE_TRIM_STEP;
        if (down  && !prevDpadDown)  distanceTrim -= DISTANCE_TRIM_STEP;
        if (left  && !prevDpadLeft)  turretTrim   += TURRET_TRIM_STEP;
        if (right && !prevDpadRight) turretTrim   -= TURRET_TRIM_STEP;

        prevDpadUp = up;
        prevDpadDown = down;
        prevDpadLeft = left;
        prevDpadRight = right;

        // Their teleop trims the shot range through this same field.
        AutoAimSubsystem.SHOT_DISTANCE_OFFSET = distanceTrim;
    }

    private void handleToggles() {
        if (gamepad1.left_bumper && !prevLeftBumper1) {
            aimOn = !aimOn;
        }
        prevLeftBumper1 = gamepad1.left_bumper;

        if (gamepad2.left_bumper && !prevLeftBumper2) {
            autoTurret = !autoTurret;
        }
        prevLeftBumper2 = gamepad2.left_bumper;

        if (gamepad1.b && !prevB1) {
            fieldCentric = !fieldCentric;
        }
        prevB1 = gamepad1.b;

        // Localization recovery: park the robot on RESEED_POSE, press OPTIONS.
        if (gamepad1.options && !prevOptions1) {
            follower.setPose(RESEED_POSE);
        }
        prevOptions1 = gamepad1.options;
    }

    /**
     * Feeds AutoAim the live Pedro pose. AutoAim drives the turret and the hood
     * itself and hands back the flywheel speed for this range.
     *
     * Their teleop applies the shooter's offset from the centre of rotation at the
     * call site (V2 tests/AASSTEST.java:83); same here.
     *
     * isShootOnTheMove is TRUE here, unlike the auto -- a driver shoots while
     * rolling, and that branch is what leads the shot for chassis velocity.
     */
    private void updateAim() {
        Pose p = follower.getPose();
        Vector v = follower.getVelocity();
        double headingDeg = Math.toDegrees(p.getHeading());
        double shooterX = p.getX() + Math.cos(p.getHeading()) * robotConstants.SHOOTER_DRIVETRAIN_OFFSET;
        double shooterY = p.getY() + Math.sin(p.getHeading()) * robotConstants.SHOOTER_DRIVETRAIN_OFFSET;

        boolean manual = !autoTurret || !aimOn;
        boolean braking = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) < 0.15;

        aim = autoAim.update(
                shooterX, shooterY,
                v.getXComponent(), v.getYComponent(),
                headingDeg,
                // Pedro reports heading rate in RADIANS/sec; AutoAim wants degrees.
                Math.toDegrees(follower.getAngularVelocity()),
                goalX(), goalY(),
                manual,
                aimOn ? MANUAL_DISTANCE : HOLD_DISTANCE,
                aimOn,      // isShootOnTheMove -- lead the shot only while aiming
                braking,
                // Parking the turret means straight ahead, so no trim applied.
                aimOn ? turretTrim : 0.0);
    }

    /**
     * Aim on  -> gates open, flywheel at the solved speed.
     * Aim off -> gates shut, flywheel idling at their shooterHold().
     *
     * Falls back to shooterHold() rather than zero when a solve has no target, so
     * a momentary bad solve cannot spin the shooter down mid-volley.
     */
    private void handleShooterAndGate() {
        if (aimOn) {
            intake.gateOpen();
            intake.intakeEngage();
            if (aim.hasTarget && aim.targetRpm > 0.0) {
                shooter.setShooterVelocity(aim.targetRpm);
            } else {
                shooter.shooterHold();
            }
        } else {
            intake.gateClose();
            intake.intakeDisengage();
            shooter.shooterHold();
        }
    }

    /**
     * Their priority order from A_1_AA_AS: collecting beats firing, and picking
     * up cancels aim mode so the driver cannot intake into an open gate.
     */
    private void handleIntake() {
        firing = false;

        if (gamepad1.right_trigger > 0.1) {
            intake.intakeIn();
            aimOn = false;
        } else if (gamepad1.left_trigger > 0.1) {
            intake.intakeOut(gamepad1.left_trigger);
        } else if (gamepad1.right_bumper) {
            // Feed power scales with the shot range, their calculateIntakePower().
            intake.intakeFire(shooter.calculateIntakePower());
            firing = true;
        } else {
            intake.intakeStop();
        }
    }

    private void report() {
        panelsTelemetry.debug("Alliance", allianceName());
        panelsTelemetry.debug("AIM ON", aimOn);
        panelsTelemetry.debug("FIRING", firing);
        panelsTelemetry.debug("Auto turret", autoTurret);
        panelsTelemetry.debug("Field centric", fieldCentric);

        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));

        panelsTelemetry.debug("Aim has target", aim.hasTarget);
        panelsTelemetry.debug("Aim LOCKED", aim.isAimLocked);
        panelsTelemetry.debug("Aim range (in)", aim.targetDist);
        panelsTelemetry.debug("Aim error (deg)", aim.aimError);
        panelsTelemetry.debug("Aim tolerance (deg)", aim.currentTolerance);
        panelsTelemetry.debug("Turret target (deg)", aim.targetTurretAngle);
        panelsTelemetry.debug("Turret actual (deg)", autoAim.getCurrentTurretAngle());
        panelsTelemetry.debug("Turret ticks", autoAim.getCurrentTick());

        panelsTelemetry.debug("HOOD cmd (percent)", aim.targetPitch);
        panelsTelemetry.debug("HOOD servo pos", autoAim.hood.getPosition());
        panelsTelemetry.debug("Shooter target", aim.targetRpm);
        panelsTelemetry.debug("Shooter actual", shooter.getShooterVelocity());
        panelsTelemetry.debug("Shooter ready", shooter.shooterReady(aim.targetRpm));
        panelsTelemetry.debug("Intake fire power", shooter.calculateIntakePower());

        panelsTelemetry.debug("TRIM distance (in)", distanceTrim);
        panelsTelemetry.debug("TRIM turret (deg)", turretTrim);
        panelsTelemetry.debug("Battery (V)", autoAim.getCurrentBatteryVoltage());
        panelsTelemetry.update(telemetry);
    }
}
