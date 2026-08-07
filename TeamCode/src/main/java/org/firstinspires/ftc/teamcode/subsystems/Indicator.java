package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Team 32008's own Indicator, copied from FTC-32008 V2 subsystems/Indicator.java.
 *
 * ONE DELIBERATE DEVIATION: init() tolerates the servo being absent.
 *
 * Their file does a bare hardwareMap.get(Servo.class, "indicator"), and "indicator"
 * is NOT in this repo's RobotConfigs -- their robot has an LED status servo this
 * one may not. A bare get() throws on a missing device, and because Robot.init()
 * calls this, that single missing servo would take down the entire TeleOp at INIT
 * rather than just losing a status light. Guarded, and every setColor() call
 * becomes a no-op when it is not there.
 *
 * If the servo does exist, add it to RobotConfigs and this behaves exactly as theirs.
 */
public class Indicator {
    /** Device name in the robot configuration. Absent on this robot -- see class comment. */
    public static final String INDICATOR = "indicator";

    private Servo indicator;

    public static class Color {
        public static double OFF = 0.0;
        public static double RED = 0.28;
        public static double ORANGE = 0.33;
        public static double YELLOW = 0.39;
        public static double SAGE = 0.44;
        public static double GREEN = 0.5;
        public static double AZURE = 0.56;
        public static double BLUE = 0.61;
        public static double INDIGO = 0.67;
        public static double VIOLET = 0.72;
        public static double WHITE = 1.0;
    }

    public void init(HardwareMap hardwareMap) {
        try {
            indicator = hardwareMap.get(Servo.class, INDICATOR);
        } catch (IllegalArgumentException e) {
            // Not configured on this robot. Status light only -- nothing else depends on it.
            indicator = null;
        }
    }

    /** True if the servo is actually present, so telemetry can say so out loud. */
    public boolean isPresent() {
        return indicator != null;
    }

    public void setColor(double color) {
        if (indicator != null) {
            indicator.setPosition(color);
        }
    }
}
