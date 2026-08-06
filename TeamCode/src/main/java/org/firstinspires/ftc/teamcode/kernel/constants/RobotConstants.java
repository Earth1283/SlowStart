package org.firstinspires.ftc.teamcode.kernel.constants;

import com.qualcomm.robotcore.util.Range;

public class RobotConstants {
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

    /** RUN_TO_POSITION treats power as a speed cap, not applied effort -- full power here. */
    public static double TURRET_HOLD_POWER = 1.0;
    public static double TURRET_YAW_TOLERANCE_DEG = 2.0;
    public static double TURRET_POSITION_PIDF_P = 20.0;

    /** Seconds the gates are held open to feed one volley. */
    public static double GATE_FEED_SECONDS = 0.9;

    /**
     * Assumed shooting distance for TeleOp, inches. TeleOp has no localizer of its
     * own, so it cannot solve distance the way auto does -- the driver dials this
     * in from Panels instead.
     */
    public static double TELEOP_SHOOT_DISTANCE = 136.0;

    /** Keeps the flywheel spinning at a low idle between shots. */
    public static double SHOOTER_HOLD_VELOCITY = 1300.0;

    public static double INTAKE_FIRE_POWER = 0.9;
    /** Gentler collection when artifacts are already stacking up. */
    public static double INTAKE_SLOW_POWER = 0.35;

    // Cubic fits, velocity/hood as a function of target distance (inches):
    //     f(x) = A*x^3 + B*x^2 + C*x + D
    public static double SHOOTER_VELOCITY_A = -0.0002357526;
    public static double SHOOTER_VELOCITY_B =  0.07831055;
    public static double SHOOTER_VELOCITY_C = -1.131756;
    public static double SHOOTER_VELOCITY_D =  1236.833;
    public static double SHOOTER_MIN_VELOCITY = 1100;
    public static double SHOOTER_MAX_VELOCITY = 2500;

    public static double SHOOTER_HOOD_A =  1.101584e-7;
    public static double SHOOTER_HOOD_B = -0.0000255276;
    public static double SHOOTER_HOOD_C =  0.004903628;
    public static double SHOOTER_HOOD_D =  0.408032;
    public static double SHOOTER_MIN_HOOD_PERCENT = 0.0;
    public static double SHOOTER_MAX_HOOD_PERCENT = 1.0;

    private static double cubic(double a, double b, double c, double d, double x) {
        return a * x * x * x + b * x * x + c * x + d;
    }

    /** Flywheel speed in ticks/sec for a target at {@code distanceInches}. */
    public static double velocityForDistance(double distanceInches) {
        return Range.clip(
                cubic(SHOOTER_VELOCITY_A, SHOOTER_VELOCITY_B, SHOOTER_VELOCITY_C,
                        SHOOTER_VELOCITY_D, distanceInches),
                SHOOTER_MIN_VELOCITY, SHOOTER_MAX_VELOCITY);
    }

    /** Hood travel fraction 0..1 for a target at {@code distanceInches}. */
    public static double hoodPercentForDistance(double distanceInches) {
        return Range.clip(
                cubic(SHOOTER_HOOD_A, SHOOTER_HOOD_B, SHOOTER_HOOD_C,
                        SHOOTER_HOOD_D, distanceInches),
                SHOOTER_MIN_HOOD_PERCENT, SHOOTER_MAX_HOOD_PERCENT);
    }
}
