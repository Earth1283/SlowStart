package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs.LEFT_BACK;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs.LEFT_FRONT;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs.PIN_POINT;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs.RIGHT_BACK;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs.RIGHT_FRONT;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Team 32008's own Drivetrain, copied from FTC-32008 V2 subsystems/Drivetrain.java.
 *
 * Changes from their file, both forced rather than chosen:
 *   1. constants imports point at kernel.constants;
 *   2. their drive(GamepadManager, double) overload is dropped -- it needs
 *      com.bylazar.gamepad.GamepadManager, and nothing in their teleop calls it.
 * Every motor direction, offset, gain and formula is theirs, unedited.
 *
 * FRAME: this owns the Pinpoint directly and works in the PINPOINT frame, which
 * is what their whole teleop is written against. That is a different frame from
 * the Pedro one the auto uses -- pinX = pedroY, pinY = 144 - pedroX,
 * pinH = pedroH - 90 deg. The teleop applies that conversion once when seeding
 * from the auto's final pose; do not mix the two anywhere else.
 *
 * The pod offsets and encoder directions below match pedroPathing/Constants.java
 * (forwardPodY -141.5 mm, strafePodX 0, FORWARD/REVERSED), so both localizers
 * describe the same physical robot.
 */
public class Drivetrain {
    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;
    public GoBildaPinpointDriver pinPoint;
    private double theta, power, turn, realTheta;
    public static double headingError, currentHeading, headingCorrection, lastHeadingError,
            KPC = 0.012, KDC = 0.00075, KS = 0.07, stickMin = 0.05;
    private ElapsedTime dt = new ElapsedTime();

    public void init(HardwareMap hardwareMap) {
        pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, PIN_POINT);
        leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT);
        leftBack = hardwareMap.get(DcMotorEx.class, LEFT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT);
        rightBack = hardwareMap.get(DcMotorEx.class, RIGHT_BACK);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pinPoint.setOffsets(-141.5, 0, DistanceUnit.MM);
        pinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinPoint.resetPosAndIMU();
    }

    public void drive(Gamepad gamepad, double powerScale) {
        double y = -gamepad.left_stick_y, x = gamepad.left_stick_x, rx = gamepad.right_stick_x * 1;
        leftFront.setPower((y + x + rx) * powerScale);
        leftBack.setPower((y - x + rx) * powerScale);
        rightFront.setPower((y - x - rx) * powerScale);
        rightBack.setPower((y + x - rx) * powerScale);
    }

    public double getHeading() {
        return pinPoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    public void driveAim(Gamepad gamepad, double p) {
        double y = -gamepad.left_stick_y, x = gamepad.left_stick_x, rx = gamepad.right_stick_x * 1;
        theta = Math.atan2(y, x) * 180 / Math.PI;
        power = Math.hypot(x, y);
        turn = rx;

        realTheta = (360 - pinPoint.getPosition().getHeading(AngleUnit.DEGREES)) + theta;

        double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = (power * cos / maxSinCos + turn);
        double rightFrontPower = (power * sin / maxSinCos - turn);
        double leftBackPower = (power * sin / maxSinCos + turn);
        double rightBackPower = (power * cos / maxSinCos - turn);

        leftFront.setPower(leftFrontPower * p);
        rightFront.setPower(rightFrontPower * p);
        leftBack.setPower(leftBackPower * p);
        rightBack.setPower(rightBackPower * p);
    }

    public Pose2D getPosition() {
        return pinPoint.getPosition();
    }

    public void updatePinpoint() {
        pinPoint.update();
    }

    public void driveAim(Gamepad gamepad, boolean aim, double target) {
        double y = -gamepad.left_stick_y, x = gamepad.left_stick_x, rx = gamepad.right_stick_x * 0.9;
        y = Math.abs(y) < stickMin ? 0 : y;
        x = Math.abs(x) < stickMin ? 0 : x;
        rx = Math.abs(rx) < stickMin ? 0 : rx;
        currentHeading = (pinPoint.getHeading(AngleUnit.RADIANS) + Math.PI) % (Math.PI * 2) - Math.PI;
        headingError = target - Math.toDegrees(currentHeading);
        if (headingError >= 180)
            headingError -= 360;
        else if (headingError < -180)
            headingError += 360;
        if (Math.abs(headingError) < 1.0)
            headingCorrection = 0;
        else
            headingCorrection = recalcTurn(-(headingError * KPC
                    + (headingError - lastHeadingError) / dt.seconds() * KDC));
        dt.reset();
        lastHeadingError = headingError;
        if (aim) {
            turn = headingCorrection;
        } else {
            turn = rx;
        }
        leftFront.setPower((y + x + turn));
        leftBack.setPower((y - x + turn));
        rightFront.setPower((y - x - turn));
        rightBack.setPower((y + x - turn));
    }

    public double recalcTurn(double power) {
        return power * (1 - KS) + Math.signum(power) * KS;
    }
}
