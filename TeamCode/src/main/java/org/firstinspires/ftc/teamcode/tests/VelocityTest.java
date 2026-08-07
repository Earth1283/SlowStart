package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Team 32008's own VelocityTest, copied from FTC-32008 V2 tests/VelocityTest.java.
 *
 * ONLY changes: the constants imports point at this repo's kernel.constants,
 * whose classes were renamed to PascalCase (autoConstants -> AutoConstants and
 * so on). No logic touched.
 *
 * NOTE it resolves pedroPathing.Constants to THIS repo's tuned copy, not V2's --
 * that is deliberate: those are the constants measured on this robot.
 */
@TeleOp(name = "V2 VelocityTest", group = "32008 V2")
public class VelocityTest extends LinearOpMode {
    Robot robot = new Robot();
    double velocity = 0, position = 0.35;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, 1);

            if (gamepad1.dpadUpWasPressed()) velocity += 50;
            if (gamepad1.dpadDownWasPressed()) velocity -= 50;

            if (gamepad1.y) position += 0.005;
            if (gamepad1.a) position -= 0.005;

            if (gamepad1.right_trigger > 0) robot.intake.intakeIn();
            else robot.intake.intakeStop();

            if (gamepad1.right_bumper) robot.intake.gateOpen();
            if (gamepad1.left_bumper) robot.intake.gateClose();

            robot.shooter.setShooterVelocity(velocity);
            robot.shooter.panelTo(position);

            telemetry.addData("target velocity", velocity);
            telemetry.addData("current velocity", robot.shooter.leftShooter.getVelocity());
            telemetry.addData("position", position);
            telemetry.update();
        }
    }
}
