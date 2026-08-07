package org.firstinspires.ftc.teamcode.teleop;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Team 32008's own competition TeleOp, ported from FTC-32008 V2 tests/A_1_AA_AS.java.
 *
 * Their two alliance files (A_1 / A_2) are the same OpMode with a different target
 * and slightly different button ownership; that difference is factored into the
 * two subclasses rather than duplicated.
 *
 * Changes from their file, all forced rather than chosen:
 *   1. constants imports point at kernel.constants;
 *   2. LinearOpMode body split into an abstract base plus per-alliance subclasses;
 *   3. dead commented-out experiments (turretToDegPP, panelPos, hoodCorrection)
 *      dropped.
 * The aim math, turret command, gate logic, button map and telemetry are theirs,
 * unedited.
 *
 * AIMING: this is their INLINE solve driving Shooter.turretToDegree() in
 * RUN_TO_POSITION -- NOT AutoAimSubsystem. That is what their competition teleop
 * actually runs. AutoAimSubsystem is used by the autonomous.
 *
 * FRAME: everything here is in the PINPOINT frame, which is what their math is
 * written against and where kernel teleOpTargetX/Y are already correct as stated.
 * The one conversion is seeding from the auto's final pose, which arrives in the
 * PEDRO frame: pinX = pedroY, pinY = 144 - pedroX, pinH = pedroH - 90 deg. Their
 * own line, kept verbatim.
 *
 * POSE HANDOFF HAZARD: autoEndX/Y/H are static and survive between OpModes. Run
 * this without running the auto first and they hold the previous run's numbers.
 * The seeded pose is printed to telemetry -- check it before trusting auto-aim.
 *
 * CONTROLS -- gamepad 1
 *   left stick / right stick X   drive
 *   left bumper                  TOGGLE shooter + aim
 *   right bumper (hold)          FIRE, feeds at the distance-scaled intake power
 *   right trigger                intake in (cancels aim)
 *   left trigger                 intake out, proportional
 *   dpad up / down               shot distance trim  +/- 2 in
 *   dpad left / right            turret trim         +/- 1 deg
 * gamepad 2 shares the dpad trims and the fire button.
 */
@Configurable
public abstract class AllianceTeleOp extends LinearOpMode {

    /** Goal for this alliance, PINPOINT frame -- straight from kernel RobotConstants. */
    protected abstract double targetXForAlliance();
    protected abstract double targetYForAlliance();
    protected abstract String allianceName();

    /** Their DRIVE_SCALE. A_1_AA_AS calls drive(gamepad1, 1). */
    public static double DRIVE_SCALE = 1.0;

    /**
     * Their velocity lead: aim at where the goal will be relative to a moving
     * robot. A_1 keeps it, A_2 has it commented out; on here by default.
     */
    public static boolean USE_VELOCITY_LEAD = true;

    /** Their flight-time model for the lead, seconds per inch plus a base. */
    public static double FLIGHT_TIME_PER_INCH = 0.00575;
    public static double BASE_FLIGHT_TIME = 0.4;

    /** Their guard: past this the turret would wrap, so it parks instead. */
    public static double TURRET_MAX_OFFSET_DEG = 150.0;

    public static double DISTANCE_TRIM_STEP = 2.0;
    public static double TURRET_TRIM_STEP = 1.0;

    private final Robot robot = new Robot();

    private double targetX, targetY, vx, vy, at = 0.0;
    private int turretTargetHeading = 0;
    private double targetATAN, drivetrainHeading, distance;
    private int turretCorrection = 0;
    private double distanceCorrection = 0;
    private boolean shooterOn = false;
    private JoinedTelemetry joinedTele;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        // Their PEDRO -> PINPOINT seeding line, verbatim.
        robot.drivetrain.pinPoint.setPosition(new Pose2D(
                DistanceUnit.INCH, autoEndY, 144 - autoEndX,
                AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));

        teleOpTargetX = targetXForAlliance();
        teleOpTargetY = targetYForAlliance();
        targetX = teleOpTargetX;
        targetY = teleOpTargetY;

        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        while (opModeInInit()) {
            robot.drivetrain.updatePinpoint();
            Pose2D seeded = robot.drivetrain.getPosition();
            joinedTele.addData("alliance", allianceName());
            joinedTele.addData("indicator servo present", robot.indicator.isPresent());
            // Check these against where the robot actually sits. Stale if no auto ran.
            joinedTele.addData("seeded x", seeded.getX(DistanceUnit.INCH));
            joinedTele.addData("seeded y", seeded.getY(DistanceUnit.INCH));
            joinedTele.addData("seeded h", seeded.getHeading(AngleUnit.DEGREES));
            joinedTele.addData("target x", targetX);
            joinedTele.addData("target y", targetY);
            joinedTele.update();
        }

        if (isStopRequested()) return;

        // Their second seeding call after START, verbatim -- the Pinpoint can drift
        // during a long INIT and this re-plants it right before the match.
        robot.drivetrain.pinPoint.setPosition(new Pose2D(
                DistanceUnit.INCH, autoEndY, 144 - autoEndX,
                AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, DRIVE_SCALE);
            robot.drivetrain.updatePinpoint();

            handleIntake();
            solveAim();
            handleTrims();
            handleShooterAndGate();
            report();
        }

        robot.shooter.shooterStop();
        robot.intake.intakeStop();
        robot.intake.gateClose();
    }

    /**
     * Their priority order: collecting beats firing, and picking up cancels aim so
     * the driver cannot intake into an open gate.
     */
    private void handleIntake() {
        if (gamepad1.right_trigger > 0) {
            robot.intake.intakeIn();
            shooterOn = false;
        } else if (gamepad1.left_trigger > 0) {
            robot.intake.intakeOut(gamepad1.left_trigger);
        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            robot.intake.intakeIn(robot.shooter.calculateIntakePower());
        } else {
            robot.intake.intakeStop();
        }
    }

    /** Their inline aim solve, verbatim. All in the PINPOINT frame. */
    private void solveAim() {
        Pose2D current = robot.drivetrain.getPosition();
        double rx = current.getX(DistanceUnit.INCH);
        double ry = current.getY(DistanceUnit.INCH);
        drivetrainHeading = current.getHeading(AngleUnit.DEGREES);

        if (USE_VELOCITY_LEAD) {
            vx = robot.drivetrain.pinPoint.getVelX(DistanceUnit.INCH);
            vy = robot.drivetrain.pinPoint.getVelY(DistanceUnit.INCH);
            at = Math.abs(Math.hypot(teleOpTargetY - ry, teleOpTargetX - rx))
                    * FLIGHT_TIME_PER_INCH + BASE_FLIGHT_TIME;
            targetX = teleOpTargetX - at * vx;
            targetY = teleOpTargetY - at * vy;
        } else {
            targetX = teleOpTargetX;
            targetY = teleOpTargetY;
        }

        targetATAN = Math.toDegrees(Math.atan2(targetY - ry, targetX - rx));
        if (Math.abs(targetATAN - drivetrainHeading) <= TURRET_MAX_OFFSET_DEG) {
            turretTargetHeading = (int) (targetATAN - drivetrainHeading);
        } else {
            turretTargetHeading = 0;
        }
        distance = Math.abs(Math.hypot(targetY - ry, targetX - rx));
    }

    private void handleTrims() {
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed())
            distanceCorrection += DISTANCE_TRIM_STEP;
        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed())
            distanceCorrection -= DISTANCE_TRIM_STEP;
        if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed())
            turretCorrection += TURRET_TRIM_STEP;
        if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed())
            turretCorrection -= TURRET_TRIM_STEP;

        if (gamepad1.leftBumperWasPressed()) {
            shooterOn = !shooterOn;
        }
    }

    private void handleShooterAndGate() {
        if (shooterOn) {
            robot.intake.gateOpen();
            robot.shooter.setShooterByDis(distance + distanceCorrection);
            robot.shooter.turretToDegree(turretTargetHeading + turretCorrection);
            robot.intake.intakeEngage();
            robot.indicator.setColor(com.qualcomm.robotcore.util.Range.clip(
                    robot.shooter.shooterReady() ? 0.5 : 0.39, 0.0, 1.0));
        } else {
            robot.intake.gateClose();
            robot.shooter.shooterHold();
            robot.shooter.turretToDegree(0);
            robot.intake.intakeDisengage();
            robot.indicator.setColor(0.28);
        }
    }

    private void report() {
        Pose2D current = robot.drivetrain.getPosition();
        joinedTele.addData("alliance", allianceName());
        joinedTele.addData("x", current.getX(DistanceUnit.INCH));
        joinedTele.addData("y", current.getY(DistanceUnit.INCH));
        joinedTele.addData("h", current.getHeading(AngleUnit.DEGREES));
        joinedTele.addData("shooterOn", shooterOn);
        joinedTele.addData("target", targetATAN);
        joinedTele.addData("turretTo", turretTargetHeading);
        joinedTele.addData("turretDegree", robot.shooter.getTurretDegree());
        joinedTele.addData("distance", distance);
        joinedTele.addData("panelActPos", robot.shooter.hood.getPosition());
        joinedTele.addData("shooterT", targetVelocity);
        joinedTele.addData("shooterVL", robot.shooter.leftShooter.getVelocity());
        joinedTele.addData("shooterVR", robot.shooter.rightShooter.getVelocity());
        joinedTele.addData("shooterReady", robot.shooter.shooterReady());
        joinedTele.addData("turretCorrection", turretCorrection);
        joinedTele.addData("distanceCorrection", distanceCorrection);
        joinedTele.addData("intakePower", robot.shooter.calculateIntakePower());
        joinedTele.update();
    }
}
