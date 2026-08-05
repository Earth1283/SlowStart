package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.kernel.constants.robotConfigs;

/**
 * Roller-type {@link Intake} for team 32008 ("intake").
 *
 * Powers match 32008's own competition-verified Intake.java (FTC-32008 V2):
 * collect at 0.8, eject at full reverse. Note V2 does NOT reverse this motor's
 * direction -- the commented-out setDirection call in their source is left off.
 */
public class RollerIntake implements Intake {

    private DcMotor roller;

    @Override
    public void init(HardwareMap hardwareMap) {
        roller = hardwareMap.get(DcMotor.class, robotConfigs.INTAKE);
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void intake() {
        roller.setPower(RobotConstants.INTAKE_POWER);
    }

    @Override
    public void fire() {
        roller.setPower(RobotConstants.INTAKE_FIRE_POWER);
    }

    /** V2's intakeInSlow -- gentler collection when artifacts are already stacking up. */
    public void intakeSlow() {
        roller.setPower(RobotConstants.INTAKE_SLOW_POWER);
    }

    @Override
    public void reverse() {
        roller.setPower(-1.0);
    }

    @Override
    public void stop() {
        roller.setPower(0.0);
    }
}
