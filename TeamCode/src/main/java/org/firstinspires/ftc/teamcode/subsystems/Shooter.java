package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.kernel.constants.PanelConstants.SHOOTER_KD;
import static org.firstinspires.ftc.teamcode.kernel.constants.PanelConstants.SHOOTER_KF;
import static org.firstinspires.ftc.teamcode.kernel.constants.PanelConstants.SHOOTER_KI;
import static org.firstinspires.ftc.teamcode.kernel.constants.PanelConstants.SHOOTER_KP;
import static org.firstinspires.ftc.teamcode.kernel.constants.PanelConstants.VELOCITY_TOR;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs.HOOD;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs.LEFT_SHOOTER;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs.RIGHT_SHOOTER;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs.TURRET;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.HOOD_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.HOOD_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.TURRET_FULL_RANGE_DEGREE;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.TURRET_FULL_RANGE_ENCODER;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Team 32008's own Shooter, copied from FTC-32008 V2 subsystems/Shooter.java.
 *
 * ONLY change from their file: the constants imports point at kernel.constants
 * instead of constants. Every hardware call, mode, direction, coefficient and
 * curve below is theirs, unedited, so a future diff against V2 stays clean.
 *
 * The turret runs RUN_TO_POSITION via {@link #turretToDegree}, which is what
 * their competition auto actually uses -- AutoAimSubsystem is their TELEOP aimer
 * and is not in the auto path at all (see V2 auto/BLUE_FAR_18.java).
 */
@Configurable
public class Shooter {
    public DcMotorEx leftShooter;
    public DcMotorEx rightShooter;
    public DcMotorEx turret;
    public Servo hood;

    public static double targetVelocity = 0, targetHoodPercentage = 0, intakeDistance = 0, intakeP = 0.9;

//    y = 1236.833 - 1.131756*x + 0.07831055*x^2 - 0.0002357526*x^3;
//    y = 0.408032 + 0.004903628*x - 0.0000255276*x^2 + 1.101584e-7*x^3;

    public static double SHOOTER_VELOCITY_A = -0.0002357526;
    public static double SHOOTER_VELOCITY_B = 0.07831055;
    public static double SHOOTER_VELOCITY_C = -1.131756;
    public static double SHOOTER_VELOCITY_D = 1236.833;
    public static double SHOOTER_MIN_VELOCITY = 1100;
    public static double SHOOTER_MAX_VELOCITY = 2500;

    public static double SHOOTER_HOOD_A = 1.101584e-7;
    public static double SHOOTER_HOOD_B = -0.0000255276;
    public static double SHOOTER_HOOD_C = 0.004903628;
    public static double SHOOTER_HOOD_D = 0.408032;
    public static double SHOOTER_MIN_HOOD_PERCENT = 0.0;
    public static double SHOOTER_MAX_HOOD_PERCENT = 1.0;
    public static double AIRTIME_K = 0.003097283;
    public static double AIRTIME_G = 386.22;
    public static double TRAJECTORY_M = 1.3540990329400013;
    public static double TRAJECTORY_N = -0.4305736274862209;
    public static double TRAJECTORY_MAX_VELOCITY = 2100.0;

    public void init(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, LEFT_SHOOTER);
        rightShooter = hardwareMap.get(DcMotorEx.class, RIGHT_SHOOTER);
        hood = hardwareMap.get(Servo.class, HOOD);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        rightShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);

        turret = hardwareMap.get(DcMotorEx.class, TURRET);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setPositionPIDFCoefficients(20);
    }

    /**
     * Their init(hardwareMap, aass) overload. Pass aass = true when
     * {@link AutoAimSubsystem} is running: it then owns the turret AND the hood,
     * and this class must not grab them or the two fight over the same motor.
     */
    public void init(HardwareMap hardwareMap, boolean aass) {
        leftShooter = hardwareMap.get(DcMotorEx.class, LEFT_SHOOTER);
        rightShooter = hardwareMap.get(DcMotorEx.class, RIGHT_SHOOTER);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        rightShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);

        if (!aass) {
            turret = hardwareMap.get(DcMotorEx.class, TURRET);
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turret.setDirection(DcMotorSimple.Direction.REVERSE);
            turret.setPositionPIDFCoefficients(20);
            hood = hardwareMap.get(Servo.class, HOOD);
        }
    }

    public static double surfaceZ(double x, double y, double k, double g, double m, double n) {
        return ((k * x) * (k * x) / g) * Math.sin(2 * (m * y + n));
    }

    public static double zOfT(double t, double k, double g, double m, double n) {
        double x = TRAJECTORY_MAX_VELOCITY * t;
        double y = t;
        return surfaceZ(x, y, k, g, m, n);
    }

    public static double solveForT(double targetZ, double k, double g, double m, double n) {
        double lo = Math.max(0, -n / m);
        double hi = 0.8;

        while (zOfT(hi, k, g, m, n) < targetZ && hi < 1.0) {
            hi += 0.01;
        }

        for (int i = 0; i < 80; i++) {
            double mid = 0.5 * (lo + hi);
            double zMid = zOfT(mid, k, g, m, n);

            if (zMid < targetZ) {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        return 0.5 * (lo + hi);
    }

    public static double calculateAirtime(double rangeInches) {
        double t = solveForT(rangeInches, AIRTIME_K, AIRTIME_G, TRAJECTORY_M, TRAJECTORY_N);
        double velocity = TRAJECTORY_MAX_VELOCITY * t;
        double launchAngle = TRAJECTORY_M * t + TRAJECTORY_N;
        return (2 * AIRTIME_K * velocity * Math.sin(launchAngle)) / AIRTIME_G;
    }

    /** Zeroes the turret encoder. Their autoInit calls this -- 0 deg = turret as placed at INIT. */
    public void reset() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setPower(0);
    }

    public void setShooterVelocity(double velocity) {
        leftShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        rightShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        leftShooter.setVelocity(velocity);
        rightShooter.setVelocity(velocity);
    }

    public double getShooterVelocity() {
        return (rightShooter.getVelocity() + leftShooter.getVelocity()) / 2.0;
    }

    public boolean shooterReady(double target) {
        return Math.abs(getShooterVelocity() - target) <= VELOCITY_TOR;
    }

    public boolean shooterReady() {
        return Math.abs(getShooterVelocity() - targetVelocity) <= VELOCITY_TOR;
    }

    public void shooterHold() {
        leftShooter.setVelocity(1300);
        rightShooter.setVelocity(1300);
    }

    public void shooterStop() {
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }

    public void setShooter(double shooterVelocity, double hoodPercentage) {
        setShooterVelocity(shooterVelocity);
        setHoodPercent(hoodPercentage);
    }

    public void setShooterByDis(double distance) {
        targetVelocity = velocityForDistance(distance);
        targetHoodPercentage = hoodPercentForDistance(distance);
        setShooter(targetVelocity, targetHoodPercentage);
    }

    public double calculateIntakePower() {
//        y = 1.110636 + 0.001319851*x - 0.00003111962*x^2;
        return Range.clip(f(0.0, -0.00003111962, 0.001319851, 1.110636, intakeDistance) * intakeP, 0.35, 1.0);
    }

    public static double f(double a, double b, double c, double d, double x) {
        return a * Math.pow(x, 3) + b * Math.pow(x, 2) + c * x + d;
    }

    public static double velocityForDistance(double targetDistance) {
        intakeDistance = targetDistance;
        return Range.clip(
                f(SHOOTER_VELOCITY_A, SHOOTER_VELOCITY_B, SHOOTER_VELOCITY_C, SHOOTER_VELOCITY_D, targetDistance),
                SHOOTER_MIN_VELOCITY,
                SHOOTER_MAX_VELOCITY
        );
    }

    public static double hoodPercentForDistance(double targetDistance) {
        return Range.clip(
                f(SHOOTER_HOOD_A, SHOOTER_HOOD_B, SHOOTER_HOOD_C, SHOOTER_HOOD_D, targetDistance),
                SHOOTER_MIN_HOOD_PERCENT,
                SHOOTER_MAX_HOOD_PERCENT
        );
    }

    public static double hoodPercentToServo(double percent) {
        return percent * (HOOD_UPPER_LIMIT - HOOD_LOWER_LIMIT) + HOOD_LOWER_LIMIT;
    }

    public static double servoToHoodPercent(double servoPosition) {
        return (servoPosition - HOOD_LOWER_LIMIT) / (HOOD_UPPER_LIMIT - HOOD_LOWER_LIMIT);
    }

    public int getTurretPosition() {
        return turret.getCurrentPosition();
    }

    public double getTurretDegree() {
        return getTurretPosition() / TURRET_FULL_RANGE_ENCODER * TURRET_FULL_RANGE_DEGREE;
    }

    public void turretToDegree(double degree) {
        int position = (int) (degree * TURRET_FULL_RANGE_ENCODER / TURRET_FULL_RANGE_DEGREE);
        turret.setTargetPosition(position);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
    }

    public void panelTo(double pos) {
        hood.setPosition(pos);
    }

    public void setHoodPercent(double percent) {
        hood.setPosition(hoodPercentToServo(percent));
    }

    public double getHoodPercent() {
        return servoToHoodPercent(hood.getPosition());
    }
}
