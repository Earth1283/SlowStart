package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs.INTAKE;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs.LEFT_GATE;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs.RIGHT_GATE;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.LEFT_GATE_CLOSE;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.LEFT_GATE_OPEN;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.RIGHT_GATE_CLOSE;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.RIGHT_GATE_OPEN;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Team 32008's own Intake, copied from FTC-32008 V2 subsystems/Intake.java.
 *
 * ONLY change from their file: the constants imports point at kernel.constants
 * instead of constants. Note they own BOTH gate servos here rather than in a
 * separate gate class, and they deliberately do NOT reverse the intake motor
 * (their setDirection call is commented out in the original -- left off here too).
 */
public class Intake {
    private DcMotorEx intake;
    private Servo leftGate;
    private Servo rightGate;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, INTAKE);
        leftGate = hardwareMap.get(Servo.class, LEFT_GATE);
        rightGate = hardwareMap.get(Servo.class, RIGHT_GATE);

//        intake.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void intakeIn() {
        intake.setPower(0.8);
    }

    public void intakeIn(double power) {
        intake.setPower(power);
    }

    public void intakeFire(double power) {
        intake.setPower(power);
    }

    public void intakeInSlow() {
        intake.setPower(0.35);
    }

    public void intakeOut() {
        intake.setPower(-1);
    }

    public void intakeOut(double power) {
        intake.setPower(-power);
    }

    public void intakeStop() {
        intake.setPower(0);
    }

    public void gateOpen() {
        leftGate.setPosition(LEFT_GATE_OPEN);
        rightGate.setPosition(RIGHT_GATE_OPEN);
    }

    public void gateClose() {
        leftGate.setPosition(LEFT_GATE_CLOSE);
        rightGate.setPosition(RIGHT_GATE_CLOSE);
    }

    public void intakeEngage() {
    }

    public void intakeDisengage() {
    }
}
