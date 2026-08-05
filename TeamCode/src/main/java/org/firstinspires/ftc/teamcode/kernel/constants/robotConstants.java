package org.firstinspires.ftc.teamcode.kernel.constants;

public class robotConstants {
    public static double TURRET_FULL_RANGE_DEGREE = 360;
    public static double TURRET_FULL_RANGE_ENCODER = 1229;

    public static double SHOOTER_DRIVETRAIN_OFFSET = 61.5 / 25.4;

    public static double LEFT_GATE_OPEN = 0.431;
    public static double RIGHT_GATE_OPEN = 0.582;
    public static double LEFT_GATE_CLOSE = 0.616;
    public static double RIGHT_GATE_CLOSE = 0.390;

    public static volatile double autoEndX = 72;
    public static volatile double autoEndY = 72;
    public static volatile double autoEndH = Math.PI / 2.0;

    public static volatile double BLUE_TARGET_X = 136.0;
    public static volatile double BLUE_TARGET_Y = 136.0;
    public static volatile double RED_TARGET_X = 136.0;
    public static volatile double RED_TARGET_Y = 8.0;

    public static volatile double teleOpTargetX = 136.0;
//    public static volatile double teleOpTargetY = 138;
    public static volatile double teleOpTargetY = 136.0;

    public static double HOOD_UPPER_LIMIT = 1.0;
    public static double HOOD_LOWER_LIMIT = 0.0;

    public static double TURRET_HOLD_POWER = 0.3;
    public static double TURRET_YAW_TOLERANCE_DEG = 2.0;
    /** Parked yaw, degrees relative to wherever the turret sat at init. */
    public static double TURRET_PARK_YAW_DEG = -134.15;
    /** Parked pitch as a fraction of HOOD travel. */
    public static double TURRET_PARK_PITCH_PERCENT = 0.40;

    /** Seconds the gates are held open to feed one volley. */
    public static double GATE_FEED_SECONDS = 0.6;
}
