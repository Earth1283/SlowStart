package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

/**
 * Constants that do NOT already live in {@code kernel.constants}.
 *
 * ============================================================================
 * SINGLE SOURCE OF TRUTH: {@code kernel.constants} is copied directly from
 * 32008's competition robot and is the authority for hardware names, gate
 * positions, turret geometry, goal coordinates, shooter PIDF and tolerance.
 * Code references those classes DIRECTLY -- they are not mirrored here.
 *
 * Duplicating them into this file would create two copies of the same value
 * that drift apart silently, which is the exact failure this project keeps
 * hitting (borrowed 180/668 turret scale vs the real 360/1229).
 *
 * What remains here is only what kernel.constants does not carry:
 *   - the shooter distance->velocity / distance->hood curves, which live in
 *     32008's subsystems/Shooter.java rather than in their constants package
 *   - intake power levels, which 32008 hardcodes inside Intake.java
 *   - timings and tolerances specific to THIS auto
 * ============================================================================
 */
@Configurable
public class RobotConstants {

    // =====================================================================
    // SHOOTER DISTANCE MODEL -- ported from 32008's subsystems/Shooter.java
    // f(x) = A*x^3 + B*x^2 + C*x + D, x in inches
    //
    // NOTE: the "distance" fed to these is 32008's TUNED fire distance
    // (autoConstants.FAR_FIRE_DISTANCE etc), NOT the geometric distance to the
    // goal. Their blue far shot uses 126.5 where the true geometry is 141.7 --
    // the difference absorbs goal height, shooter offset and drag. Feeding real
    // geometry here overshoots by about 6%.
    // =====================================================================
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

    /** 32008's shooterHold() idle speed -- keeps the flywheel warm between shots. */
    public static double SHOOTER_HOLD_VELOCITY = 1300.0;

    // =====================================================================
    // INTAKE -- 32008 hardcodes these inside Intake.java
    // =====================================================================
    public static double INTAKE_POWER = 0.8;
    public static double INTAKE_SLOW_POWER = 0.35;
    /** Feed power while a volley is going through the gates. */
    public static double INTAKE_FIRE_POWER = 0.9;

    // =====================================================================
    // TURRET control -- geometry itself comes from kernel robotConstants
    // =====================================================================
    public static double TURRET_YAW_TOLERANCE_DEG = 2.0;
    /** 32008 drives the turret at full power under RUN_TO_POSITION. */
    public static double TURRET_HOLD_POWER = 1.0;
    /** 32008: turret.setPositionPIDFCoefficients(20). */
    public static double TURRET_POSITION_PIDF_P = 20.0;

    // =====================================================================
    // This auto's own timings
    // =====================================================================
    /** How long the gates stay open for one volley. */
    public static double GATE_FEED_SECONDS = 0.9;

    /**
     * Assumed fire distance for TeleOp. TeleOp has no localizer, so the driver
     * dials this in from Panels. Defaults to 32008's FAR_FIRE_DISTANCE.
     */
    public static double TELEOP_SHOOT_DISTANCE = 126.5;

    // ---------------------------------------------------------------------

    private static double cubic(double a, double b, double c, double d, double x) {
        return a * x * x * x + b * x * x + c * x + d;
    }

    /** Flywheel speed in ticks/sec for 32008's tuned fire distance. */
    public static double velocityForDistance(double fireDistance) {
        return Range.clip(
                cubic(SHOOTER_VELOCITY_A, SHOOTER_VELOCITY_B, SHOOTER_VELOCITY_C,
                      SHOOTER_VELOCITY_D, fireDistance),
                SHOOTER_MIN_VELOCITY, SHOOTER_MAX_VELOCITY);
    }

    /** Hood travel fraction 0..1 for 32008's tuned fire distance. */
    public static double hoodPercentForDistance(double fireDistance) {
        return Range.clip(
                cubic(SHOOTER_HOOD_A, SHOOTER_HOOD_B, SHOOTER_HOOD_C,
                      SHOOTER_HOOD_D, fireDistance),
                SHOOTER_MIN_HOOD_PERCENT, SHOOTER_MAX_HOOD_PERCENT);
    }
}
