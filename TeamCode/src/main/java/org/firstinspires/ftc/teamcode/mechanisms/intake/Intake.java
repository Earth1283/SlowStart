package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Intake {

    void init(HardwareMap hardwareMap);

    void intake();

    void fire();

    void reverse();

    void stop();
}
