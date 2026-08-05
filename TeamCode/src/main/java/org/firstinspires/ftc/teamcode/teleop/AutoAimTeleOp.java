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

@Configurable
public abstract class AutoAimTeleOp extends OpMode {

    protected abstract double goalX();
    protected abstract double goalY();
    protected abstract String allianceName();

    public static Pose RESEED_POSE = new Pose(56.0, 8.0, Math.toRadians(90));

    public static double DRIVE_SCALE = 1.0;
    public static double SLOW_SCALE = 0.35;
    public static double TURN_SCALE = 0.9;
    public static double STICK_DEADBAND = 0.05;

    public static double HOLD_DISTANCE = 126.5;

    public static double DISTANCE_TRIM_STEP = 2.0;
    public static double TURRET_TRIM_STEP = 1.0;

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
        handleIntake();
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
                aimOn,
                braking,
                aimOn ? turretTrim : 0.0);
    }

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

    private void handleIntake() {
        firing = false;

        if (gamepad1.right_trigger > 0.1) {
            intake.intakeIn();
            aimOn = false;
        } else if (gamepad1.left_trigger > 0.1) {
            intake.intakeOut(gamepad1.left_trigger);
        } else if (gamepad1.right_bumper) {
            // The comment below is a prime example of AI slop:
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
