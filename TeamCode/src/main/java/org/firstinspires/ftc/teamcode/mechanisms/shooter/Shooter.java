package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Shooter {

    void init(HardwareMap hardwareMap);

    void setTargetVelocity(double ticksPerSecond);

    double getCurrentVelocity();

    boolean atTargetVelocity();

    void stop();
}
