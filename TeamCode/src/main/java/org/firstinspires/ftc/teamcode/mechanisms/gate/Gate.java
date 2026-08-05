package org.firstinspires.ftc.teamcode.mechanisms.gate;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Gate {

    void init(HardwareMap hardwareMap);

    void open();

    void close();

    boolean isOpen();
}
