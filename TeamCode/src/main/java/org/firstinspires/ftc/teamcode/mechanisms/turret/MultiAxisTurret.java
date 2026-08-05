package org.firstinspires.ftc.teamcode.mechanisms.turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Two-axis {@link Turret} for team 32008: yaw motor "lt" plus pitch/hood servo
 * "panel".
 *
 * Rebuilt to match 32008's own competition-verified Shooter.java (FTC-32008 V2).
 * Three things came from there that were previously wrong or missing here:
 *
 *   1. THE MOTOR IS REVERSED. V2 calls setDirection(REVERSE) on the turret. This
 *      was absent before, which inverts the sense of every commanded angle.
 *   2. POSITIONAL PIDF P = 20, via setPositionPIDFCoefficients. Without it the
 *      turret runs the SDK default and settles slowly or hunts.
 *   3. FULL POWER under RUN_TO_POSITION, not a 0.3 holding power. RUN_TO_POSITION
 *      treats power as a speed CAP, not applied effort, so 0.3 just made every
 *      move sluggish.
 *
 * Scale is 360 deg over 1229 ticks (V2), replacing a borrowed 180/668 that made
 * every angle land ~8.7% short while telemetry reported the requested value.
 *
 * The encoder is zeroed at init, so yaw is measured RELATIVE TO WHERE THE TURRET
 * SITS WHEN INIT IS PRESSED. Park it the same way every time, or wire a homing
 * switch to the unused "rts" digital input to make it absolute.
 */
public class MultiAxisTurret implements Turret {

    private DcMotorEx yaw;
    private Servo pitch;
    private double yawTargetDeg = 0.0;

    @Override
    public void init(HardwareMap hardwareMap) {
        yaw = hardwareMap.get(DcMotorEx.class, RobotConstants.TURRET_YAW_NAME);

        yaw.setDirection(DcMotorSimple.Direction.REVERSE);
        yaw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yaw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yaw.setPositionPIDFCoefficients(RobotConstants.TURRET_POSITION_PIDF_P);
        yaw.setTargetPosition(0);
        yaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        yaw.setPower(RobotConstants.TURRET_HOLD_POWER);
        yaw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pitch = hardwareMap.get(Servo.class, RobotConstants.TURRET_PITCH_NAME);
    }

    @Override
    public void setSetpoint(double yawDegrees, double pitchPercent) {
        yawTargetDeg = yawDegrees;
        yaw.setTargetPosition((int) Math.round(degreesToTicks(yawDegrees)));
        yaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        yaw.setPower(RobotConstants.TURRET_HOLD_POWER);

        double clamped = Range.clip(pitchPercent, 0.0, 1.0);
        pitch.setPosition(RobotConstants.PITCH_MIN
                + clamped * (RobotConstants.PITCH_MAX - RobotConstants.PITCH_MIN));
    }

    /** Aims both axes for a target at {@code distanceInches}, using V2's distance model. */
    public void aimForDistance(double yawDegrees, double distanceInches) {
        setSetpoint(yawDegrees, RobotConstants.hoodPercentForDistance(distanceInches));
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
        return Math.abs(getYawDegrees() - yawTargetDeg) <= RobotConstants.TURRET_YAW_TOLERANCE_DEG;
    }

    @Override
    public void stop() {
        yaw.setPower(0.0);
    }

    private double degreesToTicks(double degrees) {
        return degrees * RobotConstants.TURRET_FULL_RANGE_ENCODER
                / RobotConstants.TURRET_FULL_RANGE_DEGREE;
    }

    private double ticksToDegrees(double ticks) {
        return ticks / RobotConstants.TURRET_FULL_RANGE_ENCODER
                * RobotConstants.TURRET_FULL_RANGE_DEGREE;
    }
}
