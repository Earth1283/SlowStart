package org.firstinspires.ftc.teamcode.mechanisms.turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.kernel.constants.robotConfigs;
import org.firstinspires.ftc.teamcode.kernel.constants.robotConstants;

public class MultiAxisTurret implements Turret {

    private DcMotorEx yaw;
    private Servo pitch;
    private double yawTargetDeg = 0.0;

    @Override
    public void init(HardwareMap hardwareMap) {
        yaw = hardwareMap.get(DcMotorEx.class, robotConfigs.TURRET);

        yaw.setDirection(DcMotorSimple.Direction.REVERSE);
        yaw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yaw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yaw.setPositionPIDFCoefficients(robotConstants.TURRET_POSITION_PIDF_P);
        yaw.setTargetPosition(0);
        yaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        yaw.setPower(robotConstants.TURRET_HOLD_POWER);
        yaw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pitch = hardwareMap.get(Servo.class, robotConfigs.HOOD);
    }

    @Override
    public void setSetpoint(double yawDegrees, double pitchPercent) {
        yawTargetDeg = yawDegrees;
        yaw.setTargetPosition((int) Math.round(degreesToTicks(yawDegrees)));
        yaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        yaw.setPower(robotConstants.TURRET_HOLD_POWER);

        double clamped = Range.clip(pitchPercent, 0.0, 1.0);
        pitch.setPosition(robotConstants.HOOD_LOWER_LIMIT
                + clamped * (robotConstants.HOOD_UPPER_LIMIT - robotConstants.HOOD_LOWER_LIMIT));
    }

    public void aimForDistance(double yawDegrees, double distanceInches) {
        setSetpoint(yawDegrees, robotConstants.hoodPercentForDistance(distanceInches));
    }

    @Override
    public void update() {
        // RUN_TO_POSITION is serviced by the hub's own controller; nothing per-loop.
    }

    @Override
    public double getYawDegrees() {
        return ticksToDegrees(yaw.getCurrentPosition());
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(getYawDegrees() - yawTargetDeg) <= robotConstants.TURRET_YAW_TOLERANCE_DEG;
    }

    @Override
    public void stop() {
        yaw.setPower(0.0);
    }

    private double degreesToTicks(double degrees) {
        return degrees * robotConstants.TURRET_FULL_RANGE_ENCODER
                / robotConstants.TURRET_FULL_RANGE_DEGREE;
    }

    private double ticksToDegrees(double ticks) {
        return ticks / robotConstants.TURRET_FULL_RANGE_ENCODER
                * robotConstants.TURRET_FULL_RANGE_DEGREE;
    }
}
