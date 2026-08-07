package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.TOTAL_SHOOT_TIME;
import static org.firstinspires.ftc.teamcode.kernel.constants.PanelConstants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.kernel.constants.PanelConstants.SHOOT_MS;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.autoEndY;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.teleOpTargetX;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.teleOpTargetY;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Team 32008's own AutoAimTest_V, copied from FTC-32008 V2 tests/AutoAimTest_V.java.
 *
 * ONLY changes: the constants imports point at this repo's kernel.constants,
 * whose classes were renamed to PascalCase (autoConstants -> AutoConstants and
 * so on). No logic touched.
 *
 * NOTE it resolves pedroPathing.Constants to THIS repo's tuned copy, not V2's --
 * that is deliberate: those are the constants measured on this robot.
 */
@TeleOp(name = "V2 AutoAimTest_V", group = "32008 V2")
public class AutoAimTest_V extends LinearOpMode {
    Robot robot = new Robot();
    double targetX = 136.5, targetY = 138;
//    double targetX = 136.5, targetY = 6;
    int turretTargetHeading = 0;
    double targetATAN, turretCurrentHeading;
    double panelPos = 0.5, shooterVelocity = 2300;
//    double panelPos = 0.555, shooterVelocity = 2150;
    boolean shooterOn = false;
    double distance;
    int turretCorrection = 0;
    double distanceCorrection = 0;
    long gap = 0;
    ElapsedTime timer = new ElapsedTime();
    boolean intakeOn = false, intakeLeft = false, triggered = false, shootAll = false;
    boolean lh = false, rh = false, yh = false, ch = false;
    JoinedTelemetry joinedTele;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
//        robot.shooter.reset();
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        targetX = teleOpTargetX;
        targetY = teleOpTargetY;
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        waitForStart();
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, 1);

            if (gamepad1.right_trigger > 0) {
                robot.intake.intakeIn();
                shooterOn = false;
            } else if (gamepad1.left_trigger > 0) {
                robot.intake.intakeOut(gamepad1.left_trigger);
            } else if (gamepad1.right_bumper) {
                robot.intake.intakeIn(INTAKE_POWER);
            } else if (gamepad1.cross && timer.milliseconds() > TOTAL_SHOOT_TIME) {
//                robot.shootAll(SHOOT_MS, robot.shooter.calculateGap());
                robot.shootAll(SHOOT_MS, gap);
                shootAll = true;
                timer.reset();
            } else if (!shootAll) {
                robot.intake.intakeStop();
            }

            if (shootAll && timer.milliseconds() > TOTAL_SHOOT_TIME) {
                shootAll = false;
            }

            Pose2D current = robot.drivetrain.getPosition();
            turretCurrentHeading = current.getHeading(AngleUnit.DEGREES);
            targetATAN = Math.toDegrees(Math.atan2((targetY - current.getY(DistanceUnit.INCH)), (targetX - current.getX(DistanceUnit.INCH))));
            if (Math.abs(targetATAN - turretCurrentHeading) <= 80) {
                turretTargetHeading = (int) (targetATAN - turretCurrentHeading);
            } else {
                turretTargetHeading = 0;
            }
            distance = Math.abs(Math.hypot(targetY - current.getY(DistanceUnit.INCH), targetX - current.getX(DistanceUnit.INCH)));

//            if (g1.getDpadUp()) distanceCorrection += 2;
//            if (g1.getDpadDown()) distanceCorrection -= 2;
//
            if (gamepad1.squareWasPressed()) turretCorrection -= 2;
            if (gamepad1.circleWasPressed()) turretCorrection += 2;

            if (gamepad1.triangle && !yh) {
                shooterOn = !shooterOn;
                yh = true;
            } else if (!gamepad1.triangle && yh) {
                yh = false;
            }

            if (shooterOn) {
                robot.intake.gateOpen();
                robot.shooter.setShooterVelocity(shooterVelocity);
//                robot.shooter.setShooterByDis(distance + distanceCorrection);
                robot.shooter.turretToDegree(turretTargetHeading + turretCorrection);
//                robot.shooter.turretToDegree(0);
            }
            else {
                robot.intake.gateClose();
                robot.shooter.shooterHold();
//                robot.shooter.shooterStop();
                robot.shooter.turretToDegree(0);
            }

            robot.shooter.panelTo(panelPos);

            if (gamepad1.dpad_up && panelPos < 1) panelPos += 0.005;
            if (gamepad1.dpad_down && panelPos > 0) panelPos -= 0.005;
            if (gamepad1.dpad_left && !lh) {
                shooterVelocity -= 20;
                lh = true;
            } else if (!gamepad1.dpad_left && lh) {
                lh = false;
            }
            if (gamepad1.dpad_right && !rh) {
                shooterVelocity += 20;
                rh = true;
            } else if (!gamepad1.dpad_right && rh) {
                rh = false;
            }

//            if (gamepad1.dpad_up) {
//                shooterVelocity = 2000;
//                panelPos = 0.575;
//                INTAKE_POWER = 0.55;
//            }
//            if (gamepad1.dpad_down) {
//                shooterVelocity = 1520;
//                panelPos = 0.57;
//                INTAKE_POWER = 1;
//            }

            joinedTele.addData("x", current.getX(DistanceUnit.INCH));
            joinedTele.addData("y", current.getY(DistanceUnit.INCH));
            joinedTele.addData("h", current.getHeading(AngleUnit.DEGREES));
            joinedTele.addData("target", targetATAN);
            joinedTele.addData("turretTo", turretTargetHeading);
            joinedTele.addData("turretTicks", robot.shooter.getTurretPosition());
            joinedTele.addData("turretDegree", robot.shooter.getTurretDegree());
            joinedTele.addData("distance", distance);
            joinedTele.addData("panel", panelPos);
            joinedTele.addData("panelActPos", robot.shooter.hood.getPosition());
            joinedTele.addData("shooterT", shooterVelocity);
            joinedTele.addData("shooterVL", robot.shooter.leftShooter.getVelocity());
            joinedTele.addData("shooterVR", robot.shooter.rightShooter.getVelocity());
            joinedTele.addData("turretCorrection", turretCorrection);
            joinedTele.addData("distanceCorrection", distanceCorrection);
            joinedTele.addData("shootAllTime", TOTAL_SHOOT_TIME);
            joinedTele.update();
        }
    }
}
