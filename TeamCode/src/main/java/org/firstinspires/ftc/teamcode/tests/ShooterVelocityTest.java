package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Team 32008's own ShooterVelocityTest, copied from FTC-32008 V2 tests/ShooterVelocityTest.java.
 *
 * ONLY changes: the constants imports point at this repo's kernel.constants,
 * whose classes were renamed to PascalCase (autoConstants -> AutoConstants and
 * so on). No logic touched.
 *
 * NOTE it resolves pedroPathing.Constants to THIS repo's tuned copy, not V2's --
 * that is deliberate: those are the constants measured on this robot.
 */
@TeleOp(name = "V2 ShooterVelocityTest", group = "32008 V2")
@Configurable
public class ShooterVelocityTest extends LinearOpMode {
    DcMotorEx leftShooter, rightShooter;
    double velocity = 1500;
    public static double kp = 0, ki = 0, kd = 0, kf = 0;
    boolean shooterOn = false;
    JoinedTelemetry joinedTele;

    @Override
    public void runOpMode() throws InterruptedException {
        leftShooter = hardwareMap.get(DcMotorEx.class, "ls");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rs");

        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.leftBumperWasPressed()) shooterOn = !shooterOn;
            if (shooterOn) {
                leftShooter.setVelocityPIDFCoefficients(kp, ki, kd, kf);
                rightShooter.setVelocityPIDFCoefficients(kp, ki, kd, kf);
                leftShooter.setVelocity(velocity);
                rightShooter.setVelocity(velocity);
            } else {
                leftShooter.setPower(0);
                rightShooter.setPower(0);
            }

            if (gamepad1.dpadUpWasPressed()) velocity += 20;
            if (gamepad1.dpadDownWasPressed()) velocity -= 20;

            joinedTele.addData("targetVelocity", velocity);
            joinedTele.addData("actualVelocity", (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2.0);
            joinedTele.update();
        }
    }
}
