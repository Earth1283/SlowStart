package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kernel.constants.panelConstants;
import org.firstinspires.ftc.teamcode.kernel.constants.robotConfigs;
import org.firstinspires.ftc.teamcode.kernel.constants.robotConstants;

public class RollerIntake implements Intake {

    private DcMotor roller;

    @Override
    public void init(HardwareMap hardwareMap) {
        roller = hardwareMap.get(DcMotor.class, robotConfigs.INTAKE);
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void intake() {
        roller.setPower(panelConstants.INTAKE_POWER);
    }

    @Override
    public void fire() {
        roller.setPower(robotConstants.INTAKE_FIRE_POWER);
    }

    /**
     * V2's intakeFire(calculateIntakePower()) -- feed power scaled to the shot
     * distance. The far shot wants ~0.70, not the flat 0.9 {@link #fire()} uses;
     * over-feeding a volley jams the gates.
     */
    public void fire(double power) {
        roller.setPower(power);
    }

    /** V2's intakeInSlow -- gentler collection when artifacts are already stacking up. */
    public void intakeSlow() {
        roller.setPower(robotConstants.INTAKE_SLOW_POWER);
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
