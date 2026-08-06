package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs.HOOD;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs.TURRET;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.TURRET_FULL_RANGE_DEGREE;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.TURRET_FULL_RANGE_ENCODER;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Team 32008's own AutoAimSubsystem, copied from FTC-32008 V2
 * subsystems/AutoAimSubsystem.java.
 *
 * ONLY change from their file: the constants imports point at kernel.constants
 * instead of constants, and RESET_TURRET_ENCODER_ON_INIT defaults to true (see
 * below). Every gain, filter, formula and hardware call is theirs, unedited.
 *
 * THIS OWNS THE TURRET AND THE HOOD. {@link Shooter} must be started with
 * init(hardwareMap, true) alongside it, or both classes grab motor "lt" and
 * fight -- Shooter's RUN_TO_POSITION against this class's power loop.
 *
 * FRAME: the solve is frame-agnostic as long as the robot pose and the target
 * are in the SAME frame and the heading is measured from that frame's +X axis.
 * Their teleop feeds it Pinpoint coordinates; this repo feeds it Pedro
 * coordinates with the goal converted to match. Do not mix the two.
 *
 * ENCODER ZERO: their file has RESET_TURRET_ENCODER_ON_INIT = false because they
 * zero the turret through Shooter.reset() in a separate ritual. Here it defaults
 * true, so 0 deg = wherever the turret sits at INIT. Park it FORWARD.
 */
@Configurable
public class AutoAimSubsystem {
    private DcMotorEx turret;
    public Servo hood;
    private HardwareMap hardwareMap;
    private PIDFController turretPIDF;

    public static double TURRET_kP = 12.0;
    public static double TURRET_kI = 0.0;
    public static double TURRET_kD = 0.2;
    public static double TURRET_kF = 0.0;
    public static double TURRET_kV = 0.001593;
    public static double TURRET_kS = 0.029016;
    public static double TURRET_kA = 0.000006;
    public static double TURRET_kLinearBraking = 0.069324;
    public static double TURRET_kQuadraticFriction = 0.000000;

    public static double TURRET_POWER_SIGN = 1.0;
    public static double TURRET_LATENCY = 0.012;
    public static double TURRET_MAX_POWER = 1.0;
    public static double TURRET_FILTER_ALPHA = 0.8;
    public static double TURRET_VEL_FILTER_ALPHA = 0.8;
    public static double TURRET_CMD_ACCEL_FILTER_ALPHA = 0.15;
    public static double TUNING_VOLTAGE = 13.84;

    public static double CHASSIS_VEL_FILTER_ALPHA = 0.3;
    public static double CHASSIS_ACCEL_FILTER_ALPHA = 0.4;
    public static double BRAKE_PREDICTION_WEIGHT = 0.55;
    public static double FORWARD_BRAKE_DECEL = 33.09567766;
    public static double LATERAL_BRAKE_DECEL = 52.88478403;
    public static double OMEGA_FILTER_ALPHA = 0.7;
    public static double HEADING_CORRECTION_ALPHA = 0.5;

    public static double PANEL_MIN = Shooter.SHOOTER_MIN_HOOD_PERCENT;
    public static double PANEL_MAX = Shooter.SHOOTER_MAX_HOOD_PERCENT;
    public static double TURRET_MIN_SAFE_ANGLE = -175.0;
    public static double TURRET_MAX_SAFE_ANGLE = 175.0;
    public static double TURRET_CENTER_ANGLE = 0.0;
    public static boolean RESET_TURRET_ENCODER_ON_INIT = true;

    public static double BASE_FLIGHT_TIME = 0.4;
    public static double FLIGHT_TIME_PER_INCH = 0.00575;
    public static double MIN_FLIGHT_TIME = 0.35;
    public static double MAX_FLIGHT_TIME = 1.25;
    public static double MECHANICAL_SHOOT_DELAY = 0.0;
    public static double SHOT_DISTANCE_OFFSET = 0.0;

    private double initialTurretOffset = 0.0;
    private double filteredTurretRelAngle = 0.0;
    private boolean isTurretFilterInitialized = false;
    private double lastTurretRelAngle = 0.0;
    private double filteredTurretVel = 0.0;
    private double lastTargetVel = 0.0;
    private double filteredTargetAccel = 0.0;
    private long lastTime = 0;
    private long lastVoltageReadTime = 0;
    private double currentBatteryVoltage = 12.0;
    private boolean isChassisFilterInitialized = false;
    private double filteredForward = 0.0;
    private double filteredLateral = 0.0;
    private double filteredForwardAccel = 0.0;
    private double filteredLateralAccel = 0.0;
    private double lastFilteredForward = 0.0;
    private double lastFilteredLateral = 0.0;
    private double filteredRobotOmega = 0.0;
    private double smoothHeading = 0.0;
    private boolean isSmoothHeadingInit = false;

    private double lastTargetRelAngle = 0.0;
    private double lastAimError = 0.0;
    private double lastTargetAbsAngle = 0.0;
    private double lastFlightTime = 0.0;

    public static class TurretCommand {
        public boolean hasTarget = false;
        public double targetRpm = 0.0;
        public double targetPitch = 0.0;
        public boolean isAimLocked = false;
        public double currentTolerance = 1.0;
        public double targetDist = 0.0;
        public double targetTurretAngle = 0.0;
        public double aimError = 0.0;
        public double flightTime = 0.0;
    }

    private static class AimCalculator {
        private static class AimResult {
            final double dist;
            final double algYaw;
            final double rpm;
            final double pitch;
            final double flightTime;

            AimResult(double dist, double algYaw, double rpm, double pitch, double flightTime) {
                this.dist = dist;
                this.algYaw = algYaw;
                this.rpm = rpm;
                this.pitch = pitch;
                this.flightTime = flightTime;
            }
        }

        static AimResult solveAim(double futureRobotX, double futureRobotY, double targetX, double targetY, double flightTime) {
            double dx = targetX - futureRobotX;
            double dy = targetY - futureRobotY;
            double dist = Math.hypot(dx, dy) * 1.05;
            if (dist < 0.1) {
                return null;
            }
            double shotDist = Math.max(0.0, dist + SHOT_DISTANCE_OFFSET);
            return new AimResult(
                    dist,
                    Math.toDegrees(Math.atan2(dy, dx)),
                    interpolate(shotDist, 1),
                    interpolate(shotDist, 2),
                    flightTime
            );
        }

        static double interpolate(double dist, int type) {
            if (type == 1) {
                return Shooter.velocityForDistance(dist);
            }
            return Range.clip(Shooter.hoodPercentForDistance(dist), PANEL_MIN, PANEL_MAX);
        }
    }

    public AutoAimSubsystem() {
    }

    public AutoAimSubsystem(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        turret = hardwareMap.get(DcMotorEx.class, TURRET);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        if (RESET_TURRET_ENCODER_ON_INIT) {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hood = hardwareMap.get(Servo.class, HOOD);
        currentBatteryVoltage = getBatteryVoltage();
        turretPIDF = new PIDFController(TURRET_kP, TURRET_kI, TURRET_kD, TURRET_kF);
    }

    private double getBatteryVoltage() {
        double maxVoltage = 0.0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > maxVoltage) {
                maxVoltage = voltage;
            }
        }
        return Math.max(8.0, maxVoltage > 0.0 ? maxVoltage : TUNING_VOLTAGE);
    }

    public double getCurrentBatteryVoltage() {
        return currentBatteryVoltage;
    }

    public double getCurrentTurretAngle() {
        return getTurretRelativeAngle();
    }

    public double getCurrentTick() {
        return turret.getCurrentPosition();
    }

    public double getFilteredTurretAngle() {
        return filteredTurretRelAngle;
    }

    public double getFilteredTurretVelocity() {
        return filteredTurretVel;
    }

    public double getLastTargetTurretAngle() {
        return lastTargetRelAngle;
    }

    public double getLastAimError() {
        return lastAimError;
    }

    public double getLastTargetAbsAngle() {
        return lastTargetAbsAngle;
    }

    public double getLastFlightTime() {
        return lastFlightTime;
    }

    private void setPitchServo(double targetPitch) {
        hood.setPosition(Shooter.hoodPercentToServo(Range.clip(targetPitch, PANEL_MIN, PANEL_MAX)));
    }

    private double getTurretRelativeAngle() {
        return turret.getCurrentPosition() / TURRET_FULL_RANGE_ENCODER * TURRET_FULL_RANGE_DEGREE + initialTurretOffset;
    }

    private double predictAxisAccel(double velocity, double measuredAccel, boolean isBraking, double brakeDecel) {
        if (!isBraking || Math.abs(velocity) <= 0.0001) {
            return measuredAccel;
        }
        double brakeAccel = -Math.signum(velocity) * brakeDecel;
        return measuredAccel * (1.0 - BRAKE_PREDICTION_WEIGHT) + brakeAccel * BRAKE_PREDICTION_WEIGHT;
    }

    private double stopAtZero(double currentValue, double predictedValue) {
        if (currentValue > 0.0 && predictedValue < 0.0) {
            return 0.0;
        }
        if (currentValue < 0.0 && predictedValue > 0.0) {
            return 0.0;
        }
        return predictedValue;
    }

    private double normalizeDeg(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }
        while (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    public TurretCommand update(
            double robotX, double robotY, double globalVx, double globalVy,
            double currentHeadingDeg, double robotAngularVelocityDeg,
            double targetX, double targetY,
            boolean isManualMode, double manualDist,
            boolean isShootOnTheMove, boolean isBraking, double yawOffset) {

        turretPIDF.setPIDF(TURRET_kP, TURRET_kI, TURRET_kD, TURRET_kF);
        TurretCommand command = new TurretCommand();

        long currentTimeForVolts = System.nanoTime();
        if (currentTimeForVolts - lastVoltageReadTime > 250_000_000L) {
            currentBatteryVoltage = getBatteryVoltage();
            lastVoltageReadTime = currentTimeForVolts;
        }

        long currentTimeNanos = System.nanoTime();
        double dt = lastTime == 0 ? 0.0001 : (currentTimeNanos - lastTime) / 1e9;
        lastTime = currentTimeNanos;

        updateTurretFilters(dt);

        updateHeadingFilter(currentHeadingDeg, robotAngularVelocityDeg, dt);
        updateChassisFilters(globalVx, globalVy, isShootOnTheMove, dt);

        double headingRad = Math.toRadians(smoothHeading);
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        double effectiveFieldVx = filteredForward * cosH - filteredLateral * sinH;
        double effectiveFieldVy = filteredForward * sinH + filteredLateral * cosH;

        AimCalculator.AimResult aimResult;
        if (isManualMode) {
            double manualRpm = AimCalculator.interpolate(manualDist, 1);
            double manualPitch = AimCalculator.interpolate(manualDist, 2);
            aimResult = new AimCalculator.AimResult(manualDist, smoothHeading, manualRpm, manualPitch, 0.0);
        } else {
            double currentDistToTarget = Math.hypot(targetX - robotX, targetY - robotY) * 1.05;
            double flightTime = Shooter.calculateAirtime(currentDistToTarget);

            double predictedForwardAccel = 0.0;
            double predictedLateralAccel = 0.0;
            if (isShootOnTheMove) {
                predictedForwardAccel = predictAxisAccel(filteredForward, filteredForwardAccel, isBraking, FORWARD_BRAKE_DECEL);
                predictedLateralAccel = predictAxisAccel(filteredLateral, filteredLateralAccel, isBraking, LATERAL_BRAKE_DECEL);
            }

            double releaseForward = filteredForward + predictedForwardAccel * MECHANICAL_SHOOT_DELAY;
            double releaseLateral = filteredLateral + predictedLateralAccel * MECHANICAL_SHOOT_DELAY;
            if (isBraking) {
                releaseForward = stopAtZero(filteredForward, releaseForward);
                releaseLateral = stopAtZero(filteredLateral, releaseLateral);
            }

            double avgForward = 0.5 * (filteredForward + releaseForward);
            double avgLateral = 0.5 * (filteredLateral + releaseLateral);
            double releaseX = robotX + (avgForward * cosH - avgLateral * sinH) * MECHANICAL_SHOOT_DELAY;
            double releaseY = robotY + (avgForward * sinH + avgLateral * cosH) * MECHANICAL_SHOOT_DELAY;

            double releaseFieldVx = releaseForward * cosH - releaseLateral * sinH;
            double releaseFieldVy = releaseForward * sinH + releaseLateral * cosH;
            double futureX = releaseX + releaseFieldVx * flightTime;
            double futureY = releaseY + releaseFieldVy * flightTime;

            aimResult = AimCalculator.solveAim(futureX, futureY, targetX, targetY, flightTime);
        }

        if (aimResult == null) {
            turret.setPower(0.0);
            return command;
        }

        command.hasTarget = true;
        command.targetRpm = aimResult.rpm;
        command.targetPitch = aimResult.pitch;
        command.targetDist = aimResult.dist;
        command.flightTime = aimResult.flightTime;
        lastFlightTime = aimResult.flightTime;

        command.currentTolerance = calculateTolerance(aimResult.dist);

        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double distSq = dx * dx + dy * dy;
        double translationalOmegaDeg = 0.0;
        if (distSq > 0.001) {
            double omegaRad = (-dx * effectiveFieldVy + dy * effectiveFieldVx) / distSq;
            translationalOmegaDeg = Math.toDegrees(omegaRad);
        }

        double compensatedTargetAbsAngle = aimResult.algYaw + yawOffset + translationalOmegaDeg * TURRET_LATENCY;
        double currentTurretAbsAngle = smoothHeading + filteredTurretRelAngle;
        double error = normalizeDeg(compensatedTargetAbsAngle - currentTurretAbsAngle);
        double targetTurretRelAngle = filteredTurretRelAngle + error;

        lastTargetAbsAngle = compensatedTargetAbsAngle;
        lastAimError = error;
        lastTargetRelAngle = targetTurretRelAngle;
        command.targetTurretAngle = targetTurretRelAngle;
        command.aimError = error;

        if (targetTurretRelAngle < TURRET_MIN_SAFE_ANGLE || targetTurretRelAngle > TURRET_MAX_SAFE_ANGLE) {
            command.hasTarget = false;
            command.isAimLocked = false;
            command.targetTurretAngle = TURRET_CENTER_ANGLE;
            driveTurretToAngle(filteredTurretRelAngle, TURRET_CENTER_ANGLE, -filteredRobotOmega, dt);
            return command;
        }

        command.isAimLocked = Math.abs(error) <= command.currentTolerance;

        double predictedRelAngle = filteredTurretRelAngle;
        double brakingDist = TURRET_kLinearBraking * Math.abs(filteredTurretVel)
                + TURRET_kQuadraticFriction * filteredTurretVel * filteredTurretVel;
        if (Math.signum(error) != Math.signum(filteredTurretVel)) {
            predictedRelAngle += Math.signum(filteredTurretVel) * brakingDist;
        }

        driveTurretToAngle(predictedRelAngle, targetTurretRelAngle, -filteredRobotOmega + translationalOmegaDeg, dt);
        setPitchServo(command.targetPitch);
        return command;
    }

    private void updateTurretFilters(double dt) {
        double rawTurretRelAngle = getTurretRelativeAngle();
        if (!isTurretFilterInitialized) {
            filteredTurretRelAngle = rawTurretRelAngle;
            lastTurretRelAngle = rawTurretRelAngle;
            filteredTurretVel = 0.0;
            lastTargetVel = 0.0;
            filteredTargetAccel = 0.0;
            isTurretFilterInitialized = true;
        } else {
            filteredTurretRelAngle = TURRET_FILTER_ALPHA * rawTurretRelAngle
                    + (1.0 - TURRET_FILTER_ALPHA) * filteredTurretRelAngle;
            if (dt > 0.0001) {
                double rawVel = (rawTurretRelAngle - lastTurretRelAngle) / dt;
                filteredTurretVel = TURRET_VEL_FILTER_ALPHA * rawVel
                        + (1.0 - TURRET_VEL_FILTER_ALPHA) * filteredTurretVel;
            }
        }
        lastTurretRelAngle = rawTurretRelAngle;
    }

    private void updateHeadingFilter(double currentHeadingDeg, double robotAngularVelocityDeg, double dt) {
        if (!isSmoothHeadingInit) {
            smoothHeading = currentHeadingDeg;
            filteredRobotOmega = robotAngularVelocityDeg;
            isSmoothHeadingInit = true;
            return;
        }

        filteredRobotOmega += OMEGA_FILTER_ALPHA * (robotAngularVelocityDeg - filteredRobotOmega);
        smoothHeading += filteredRobotOmega * dt;
        double headingError = normalizeDeg(currentHeadingDeg - smoothHeading);
        smoothHeading = normalizeDeg(smoothHeading + HEADING_CORRECTION_ALPHA * headingError);
    }

    private void updateChassisFilters(double globalVx, double globalVy, boolean isShootOnTheMove, double dt) {
        double headingRad = Math.toRadians(smoothHeading);
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        double rawForward = globalVx * cosH + globalVy * sinH;
        double rawLateral = -globalVx * sinH + globalVy * cosH;

        if (!isChassisFilterInitialized || !isShootOnTheMove) {
            filteredForward = rawForward;
            filteredLateral = rawLateral;
            filteredForwardAccel = 0.0;
            filteredLateralAccel = 0.0;
            lastFilteredForward = rawForward;
            lastFilteredLateral = rawLateral;
            isChassisFilterInitialized = true;
            return;
        }

        filteredForward += CHASSIS_VEL_FILTER_ALPHA * (rawForward - filteredForward);
        filteredLateral += CHASSIS_VEL_FILTER_ALPHA * (rawLateral - filteredLateral);
        if (dt > 0.0001) {
            double rawForwardAccel = (filteredForward - lastFilteredForward) / dt;
            double rawLateralAccel = (filteredLateral - lastFilteredLateral) / dt;
            filteredForwardAccel += CHASSIS_ACCEL_FILTER_ALPHA * (rawForwardAccel - filteredForwardAccel);
            filteredLateralAccel += CHASSIS_ACCEL_FILTER_ALPHA * (rawLateralAccel - filteredLateralAccel);
        }
        lastFilteredForward = filteredForward;
        lastFilteredLateral = filteredLateral;

    }

    private double calculateTolerance(double dist) {
        if (dist <= 20.0) {
            return 30.0;
        }
        if (dist >= 90.0) {
            return 5.0;
        }
        return 30.0 + (5.0 - 30.0) / (90.0 - 20.0) * (dist - 20.0);
    }

    private void driveTurretToAngle(double currentAngle, double targetAngle, double feedforwardVel, double dt) {
        targetAngle = Range.clip(targetAngle, -TURRET_FULL_RANGE_DEGREE, TURRET_FULL_RANGE_DEGREE);
        double pidOutputVel = turretPIDF.calculate(currentAngle, targetAngle);
        double finalTargetVel = pidOutputVel + feedforwardVel;

        double rawTargetAccel = 0.0;
        if (dt > 0.0001) {
            rawTargetAccel = (finalTargetVel - lastTargetVel) / dt;
        }
        lastTargetVel = finalTargetVel;
        filteredTargetAccel = TURRET_CMD_ACCEL_FILTER_ALPHA * rawTargetAccel
                + (1.0 - TURRET_CMD_ACCEL_FILTER_ALPHA) * filteredTargetAccel;

        double ksSign = Math.abs(finalTargetVel) > 0.5 ? Math.signum(finalTargetVel) : 0.0;
        double turretPower = filteredTargetAccel * TURRET_kA
                + finalTargetVel * TURRET_kV
                + ksSign * TURRET_kS;

        turretPower *= TUNING_VOLTAGE / currentBatteryVoltage;
        turret.setPower(Range.clip(TURRET_POWER_SIGN * turretPower, -TURRET_MAX_POWER, TURRET_MAX_POWER));
    }

    public void stop() {
        turret.setPower(0.0);
        isTurretFilterInitialized = false;
        isChassisFilterInitialized = false;
        isSmoothHeadingInit = false;
        lastTime = 0;
    }
}
