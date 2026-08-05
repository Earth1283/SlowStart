package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kernel.constants.panelConstants;
import org.firstinspires.ftc.teamcode.kernel.constants.robotConfigs;
import org.firstinspires.ftc.teamcode.kernel.constants.robotConstants;

public class DualFlywheelShooter implements Shooter {

    private DcMotorEx left, right;
    private double targetTicksPerSecond = 0.0;

    @Override
    public void init(HardwareMap hardwareMap) {
        left  = hardwareMap.get(DcMotorEx.class, robotConfigs.LEFT_SHOOTER);
        right = hardwareMap.get(DcMotorEx.class, robotConfigs.RIGHT_SHOOTER);

        left.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        applyPidf();
    }

    private void applyPidf() {
        left.setVelocityPIDFCoefficients(panelConstants.SHOOTER_KP, panelConstants.SHOOTER_KI,
                panelConstants.SHOOTER_KD, panelConstants.SHOOTER_KF);
        right.setVelocityPIDFCoefficients(panelConstants.SHOOTER_KP, panelConstants.SHOOTER_KI,
                panelConstants.SHOOTER_KD, panelConstants.SHOOTER_KF);
    }

    @Override
    public void setTargetVelocity(double ticksPerSecond) {
        applyPidf();
        targetTicksPerSecond = ticksPerSecond;
        left.setVelocity(ticksPerSecond);
        right.setVelocity(ticksPerSecond);
    }

    public void setForDistance(double distanceInches) {
        setTargetVelocity(robotConstants.velocityForDistance(distanceInches));
    }

    public void hold() {
        setTargetVelocity(robotConstants.SHOOTER_HOLD_VELOCITY);
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
                <= panelConstants.VELOCITY_TOR;
    }

    @Override
    public void stop() {
        targetTicksPerSecond = 0.0;
        left.setPower(0.0);
        right.setPower(0.0);
    }
}
