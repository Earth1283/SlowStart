package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * DECODE mechanism interface: brings artifacts into the robot, and feeds them
 * into the shooter. Derived from season-extensions/decode-2025-26.yaml's
 * {@code intake} axis; team 32008's confirmed config selects {@code roller}.
 *
 * {@link #fire()} is a distinct verb from {@link #intake()} on purpose: 32008's
 * own code (FTC-32008 V2) runs the roller at a DIFFERENT, higher power to push
 * artifacts through the gates during a volley than it uses to collect them off
 * the floor. Collapsing the two loses that distinction and under-feeds the shot.
 */
public interface Intake {

    /** Acquire hardware from the hardware map. */
    void init(HardwareMap hardwareMap);

    /** Run the intake inward to collect artifacts. */
    void intake();

    /** Run the intake at feed power, pushing artifacts into the shooter. */
    void fire();

    /** Run the intake outward (clear a jam / eject). */
    void reverse();

    /** Stop the intake. */
    void stop();
}
