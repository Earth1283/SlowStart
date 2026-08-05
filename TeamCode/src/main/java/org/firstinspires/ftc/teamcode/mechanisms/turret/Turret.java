package org.firstinspires.ftc.teamcode.mechanisms.turret;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Turret {

    void init(HardwareMap hardwareMap);

    void setSetpoint(double yawDegrees, double pitchPercent);

    void update();

    double getYawDegrees();

    boolean atSetpoint();

    void stop();
}
