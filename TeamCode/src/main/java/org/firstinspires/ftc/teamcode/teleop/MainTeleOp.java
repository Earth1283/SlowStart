package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.kernel.constants.robotConfigs;
import org.firstinspires.ftc.teamcode.kernel.constants.robotConstants;
import org.firstinspires.ftc.teamcode.mechanisms.gate.DualServoGate;
import org.firstinspires.ftc.teamcode.mechanisms.gate.Gate;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.RollerIntake;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.DualFlywheelShooter;
import org.firstinspires.ftc.teamcode.mechanisms.turret.MultiAxisTurret;

@TeleOp(name = "32008 TeleOp", group = "32008")
public class MainTeleOp extends LinearOpMode {

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    private final DualFlywheelShooter shooter = new DualFlywheelShooter();
    private final Intake intake = new RollerIntake();
    private final Gate gate = new DualServoGate();
    private final MultiAxisTurret turret = new MultiAxisTurret();

    private TelemetryManager panelsTelemetry;

    private static final double SLOW_SCALE = 0.35;
    private static final double YAW_NUDGE_DEG = 0.75;
    private static final double PITCH_NUDGE = 0.005;

    private boolean shooterOn = false;
    private boolean prevRightBumper = false;

    // Turret starts centred on its init position; hood comes from the same
    // distance model the auto uses, so a TeleOp shot matches an auto shot.
    private double turretYawDeg = 0.0;
    private double turretPitch =
            RobotConstants.hoodPercentForDistance(RobotConstants.TELEOP_SHOOT_DISTANCE);

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        leftFront  = hardwareMap.get(DcMotorEx.class, robotConfigs.LEFT_FRONT);
        leftBack   = hardwareMap.get(DcMotorEx.class, robotConfigs.LEFT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, robotConfigs.RIGHT_FRONT);
        rightBack  = hardwareMap.get(DcMotorEx.class, robotConfigs.RIGHT_BACK);

        // Same directions as pedroPathing/Constants.java and 19859's teleop.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        gate.init(hardwareMap);
        turret.init(hardwareMap);

        panelsTelemetry.debug("Status", "Initialized -- 32008 TeleOp");
        panelsTelemetry.update(telemetry);

        waitForStart();

        turret.setSetpoint(turretYawDeg, turretPitch);

        while (opModeIsActive()) {
            drive();
            handleIntake();
            handleShooterAndGate();
            handleTurret();

            turret.update();
            report();
        }

        shooter.stop();
        intake.stop();
        turret.stop();
    }

    private void drive() {
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        double lf = y + x + rx;
        double lb = y - x + rx;
        double rf = y - x - rx;
        double rb = y + x - rx;

        double denom = Math.max(1.0,
                Math.max(Math.abs(lf), Math.max(Math.abs(lb),
                        Math.max(Math.abs(rf), Math.abs(rb)))));

        double scale = gamepad1.left_bumper ? SLOW_SCALE : 1.0;

        leftFront.setPower(lf / denom * scale);
        leftBack.setPower(lb / denom * scale);
        rightFront.setPower(rf / denom * scale);
        rightBack.setPower(rb / denom * scale);
    }

    private void handleIntake() {
        if (gamepad1.right_trigger > 0.1) {
            intake.intake();
        } else if (gamepad1.left_trigger > 0.1) {
            intake.reverse();
        } else {
            intake.stop();
        }
    }

    private void handleShooterAndGate() {
        boolean rightBumper = gamepad1.right_bumper;
        if (rightBumper && !prevRightBumper) {
            shooterOn = !shooterOn;
            if (shooterOn) {
                shooter.setForDistance(RobotConstants.TELEOP_SHOOT_DISTANCE);
            } else {
                shooter.stop();
            }
        }
        prevRightBumper = rightBumper;
        if (gamepad1.a && shooter.atTargetVelocity()) {
            gate.open();
            intake.fire();
        } else {
            gate.close();
        }
    }

    private void handleTurret() {
        if (gamepad1.b) {
            turretYawDeg = 0.0;
            turretPitch  = RobotConstants.hoodPercentForDistance(
                    RobotConstants.TELEOP_SHOOT_DISTANCE);
        }

        if (gamepad1.dpad_left)  turretYawDeg += YAW_NUDGE_DEG;
        if (gamepad1.dpad_right) turretYawDeg -= YAW_NUDGE_DEG;
        if (gamepad1.dpad_up)    turretPitch  += PITCH_NUDGE;
        if (gamepad1.dpad_down)  turretPitch  -= PITCH_NUDGE;

        turretYawDeg = Range.clip(turretYawDeg,
                -robotConstants.TURRET_FULL_RANGE_DEGREE,
                 robotConstants.TURRET_FULL_RANGE_DEGREE);
        turretPitch = Range.clip(turretPitch, 0.0, 1.0);

        turret.setSetpoint(turretYawDeg, turretPitch);
    }

    private void report() {
        panelsTelemetry.debug("Shooter on", shooterOn);
        panelsTelemetry.debug("Shooter ticks/sec", shooter.getCurrentVelocity());
        panelsTelemetry.debug("Shooter at speed", shooter.atTargetVelocity());
        panelsTelemetry.debug("Gate open", gate.isOpen());
        panelsTelemetry.debug("Turret yaw cmd (deg)", turretYawDeg);
        panelsTelemetry.debug("Turret yaw actual (deg)", turret.getYawDegrees());
        panelsTelemetry.debug("Turret pitch", turretPitch);
        panelsTelemetry.update(telemetry);
    }
}
