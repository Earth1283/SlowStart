package org.firstinspires.ftc.teamcode.teleop.v2;

import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.BLUE_TARGET_X;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.BLUE_TARGET_Y;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.autoEndY;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.teleOpTargetX;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.teleOpTargetY;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.targetVelocity;

import com.bylazar.configurables.annotations.Configurable;
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
 * Team 32008's own TeleOp, copied byte-for-byte from FTC-32008 V2 tests/A_2_AA_AS_B.java.
 *
 * BLUE alliance, driver-2 buttons, velocity lead OFF, shooterStop when off.
 *
 * ONLY changes: package, the constants import path, and a name/group on @TeleOp so
 * these are findable on the picker. Not one line of logic is touched -- dead locals,
 * commented-out experiments and all. Diff this against V2 and expect zero drift.
 *
 * Frame is PINPOINT throughout. See teleop/AllianceTeleOp.java for the maintained
 * variant of the same OpMode.
 */
@TeleOp(name = "V2 A_2_AA_AS_B", group = "32008 V2")
@Configurable
public class A_2_AA_AS_B extends LinearOpMode {
    Robot robot = new Robot();
    double targetX = 136.5, targetY = 138, vx, vy;
//    double targetX = 136.5, targetY = 6;
    int turretTargetHeading = 0;
    double targetATAN, drivetrainHeading;
//    double panelPos = 0.5, shooterVelocity = 2300;
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
    public static double at = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
//        robot.shooter.reset();
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
//        robot.shooter.setPPHeading(0);
        teleOpTargetX = BLUE_TARGET_X;
        teleOpTargetY = BLUE_TARGET_Y;
        targetX = teleOpTargetX;
        targetY = teleOpTargetY;
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        waitForStart();
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
//        robot.shooter.setPPHeading(0);

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, 1);

            if (gamepad1.right_trigger > 0) {
                robot.intake.intakeIn();
                shooterOn = false;
            } else if (gamepad1.left_trigger > 0) {
                robot.intake.intakeOut(gamepad1.left_trigger);
            } else if (gamepad2.right_bumper) {
//                if (timer.milliseconds() > 180) hoodCorrection = 0.045;
                robot.intake.intakeIn(robot.shooter.calculateIntakePower());
            } else {
                robot.intake.intakeStop();
            }

            Pose2D current = robot.drivetrain.getPosition();
            drivetrainHeading = current.getHeading(AngleUnit.DEGREES);
//            vx = robot.drivetrain.pinPoint.getVelX(DistanceUnit.INCH);
//            vy = robot.drivetrain.pinPoint.getVelY(DistanceUnit.INCH);
//            at = Math.abs(Math.hypot(teleOpTargetY - current.getY(DistanceUnit.INCH), teleOpTargetX - current.getX(DistanceUnit.INCH))) * 0.00575 + 0.4;
//            targetX = teleOpTargetX - at * vx;
//            targetY = teleOpTargetY - at * vy;
            targetATAN = Math.toDegrees(Math.atan2((targetY - current.getY(DistanceUnit.INCH)), (targetX - current.getX(DistanceUnit.INCH))));
            if (Math.abs(targetATAN - drivetrainHeading) <= 150) {
                turretTargetHeading = (int) (targetATAN - drivetrainHeading);
            } else {
                turretTargetHeading = 0;
            }
            distance = Math.abs(Math.hypot(targetY - current.getY(DistanceUnit.INCH), targetX - current.getX(DistanceUnit.INCH)));

            if (gamepad2.dpadUpWasPressed()) distanceCorrection += 2;
            if (gamepad2.dpadDownWasPressed()) distanceCorrection -= 2;

            if (gamepad2.dpadLeftWasPressed()) turretCorrection += 1;
            if (gamepad2.dpadRightWasPressed()) turretCorrection -= 1;

            if (gamepad1.leftBumperWasPressed()) {
                shooterOn = !shooterOn;
//                if (shooterOn) {
//                    hoodCorrection = 0;
//                }
            }

            if (shooterOn) {
                robot.intake.gateOpen();
//                robot.shooter.setShooterVelocity(shooterVelocity);
                robot.shooter.setShooterByDis(distance + distanceCorrection);
                robot.shooter.turretToDegree(turretTargetHeading + turretCorrection);
                robot.intake.intakeEngage();
//                robot.shooter.turretToDegPP(turretTargetHeading);
            }
            else {
                robot.intake.gateClose();
//                robot.shooter.shooterHold();
                robot.shooter.shooterStop();
//                robot.shooter.turretToDegPP(drivetrainHeading);
                robot.shooter.turretToDegree(0);
                robot.intake.intakeDisengage();
            }

//            robot.shooter.panelTo(panelPos);

            joinedTele.addData("x", current.getX(DistanceUnit.INCH));
            joinedTele.addData("y", current.getY(DistanceUnit.INCH));
            joinedTele.addData("h", current.getHeading(AngleUnit.DEGREES));
            joinedTele.addData("target", targetATAN);
//            joinedTele.addData("turretTarget", turretTargetHeading);
//            joinedTele.addData("turretHeading", robot.shooter.getPPHeading());
            joinedTele.addData("turretTo", turretTargetHeading);
//            joinedTele.addData("turretTicks", robot.shooter.getTurretPosition());
            joinedTele.addData("turretDegree", robot.shooter.getTurretDegree());
            joinedTele.addData("distance", distance);
//            joinedTele.addData("panel", panelPos);
//            joinedTele.addData("panelActPos", robot.shooter.panel.getPosition());
            joinedTele.addData("shooterT", targetVelocity);
            joinedTele.addData("shooterVL", robot.shooter.leftShooter.getVelocity());
            joinedTele.addData("shooterVR", robot.shooter.rightShooter.getVelocity());
            joinedTele.addData("turretCorrection", turretCorrection);
            joinedTele.addData("distanceCorrection", distanceCorrection);
            joinedTele.addData("intakePower", robot.shooter.calculateIntakePower());
//            joinedTele.addData("shootAllTime", TOTAL_SHOOT_TIME);
            joinedTele.update();
        }
    }
}
