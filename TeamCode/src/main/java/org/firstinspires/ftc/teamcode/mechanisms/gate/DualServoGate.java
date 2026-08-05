package org.firstinspires.ftc.teamcode.mechanisms.gate;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kernel.constants.robotConfigs;
import org.firstinspires.ftc.teamcode.kernel.constants.robotConstants;

public class DualServoGate implements Gate {

    private Servo leftGate, rightGate;
    private boolean open = false;

    @Override
    public void init(HardwareMap hardwareMap) {
        leftGate  = hardwareMap.get(Servo.class, robotConfigs.LEFT_GATE);
        rightGate = hardwareMap.get(Servo.class, robotConfigs.RIGHT_GATE);
        close();
    }

    @Override
    public void open() {
        leftGate.setPosition(robotConstants.LEFT_GATE_OPEN);
        rightGate.setPosition(robotConstants.RIGHT_GATE_OPEN);
        open = true;
    }

    @Override
    public void close() {
        leftGate.setPosition(robotConstants.LEFT_GATE_CLOSE);
        rightGate.setPosition(robotConstants.RIGHT_GATE_CLOSE);
        open = false;
    }

    @Override
    public boolean isOpen() {
        return open;
    }
}
