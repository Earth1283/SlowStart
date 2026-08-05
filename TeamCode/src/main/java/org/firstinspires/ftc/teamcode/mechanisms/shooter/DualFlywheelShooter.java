package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Two-motor flywheel {@link Shooter} for team 32008 ("ls" and "rs").
 *
 * Matches 32008's own competition-verified Shooter.java (FTC-32008 V2): left
 * motor reversed, RUN_USING_ENCODER, FLOAT at zero power so a spinning flywheel
 * is never braked hard, and velocity read as the MEAN of both motors.
 *
 * Velocity PIDF is re-applied on every setTargetVelocity(), exactly as V2 does.
 * That looks redundant but is deliberate: it means a coefficient edited live in
 * Panels takes effect on the next command instead of requiring a restart.
 *
 * Speeds are ENCODER TICKS PER SECOND throughout -- what setVelocity() natively
 * takes, so no counts-per-revolution conversion exists to get wrong.
 */
public class DualFlywheelShooter implements Shooter {

    private DcMotorEx left, right;
    private double targetTicksPerSecond = 0.0;

    @Override
    public void init(HardwareMap hardwareMap) {
        left  = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTER_LEFT_NAME);
        right = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTER_RIGHT_NAME);

        left.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        applyPidf();
    }

    private void applyPidf() {
        left.setVelocityPIDFCoefficients(RobotConstants.SHOOTER_KP, RobotConstants.SHOOTER_KI,
                RobotConstants.SHOOTER_KD, RobotConstants.SHOOTER_KF);
        right.setVelocityPIDFCoefficients(RobotConstants.SHOOTER_KP, RobotConstants.SHOOTER_KI,
                RobotConstants.SHOOTER_KD, RobotConstants.SHOOTER_KF);
    }

    @Override
    public void setTargetVelocity(double ticksPerSecond) {
        applyPidf();
        targetTicksPerSecond = ticksPerSecond;
        left.setVelocity(ticksPerSecond);
        right.setVelocity(ticksPerSecond);
    }

    /** Commands the speed V2's model wants for a target at {@code distanceInches}. */
    public void setForDistance(double distanceInches) {
        setTargetVelocity(RobotConstants.velocityForDistance(distanceInches));
    }

    /**
     * Keeps the flywheel spinning at a low idle between shots (V2's shooterHold).
     * Cheaper than spinning up from zero every time and easier on the battery
     * than holding full shooting speed across the whole match.
     */
    public void hold() {
        setTargetVelocity(RobotConstants.SHOOTER_HOLD_VELOCITY);
    }

    @Override
    public double getCurrentVelocity() {
        return (left.getVelocity() + right.getVelocity()) / 2.0;
    }

    @Override
    public boolean atTargetVelocity() {
        // A zero target is never "at speed", so nothing can feed into a stopped shooter.
        if (targetTicksPerSecond <= 0.0) {
            return false;
        }
        return Math.abs(getCurrentVelocity() - targetTicksPerSecond)
                <= RobotConstants.SHOOTER_VELOCITY_TOLERANCE;
    }

    @Override
    public void stop() {
        targetTicksPerSecond = 0.0;
        left.setPower(0.0);
        right.setPower(0.0);
    }
}
