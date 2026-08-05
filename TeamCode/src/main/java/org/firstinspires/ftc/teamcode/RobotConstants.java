package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

/**
 * Live-tunable constants for team 32008.
 *
 * ============================================================================
 * SOURCE: every value here now comes from 32008's competition-verified
 * "FTC-32008 V2" repository -- this team's own robot, not another team's.
 * Nothing in this file is borrowed from 19859 any more, and nothing is a
 * placeholder.
 * ============================================================================
 *
 * Robot configuration this maps to:
 *   Control Hub   motors  lt, intake, lb, lf
 *                 servos  indicator, rg, lg, panel
 *                 digital rts
 *                 I2C     pp   (bus 0 -- see pedroPathing/Constants.java)
 *   Expansion Hub motors  ls, rs, rb, rf
 *
 * "indicator" and "rts" are deliberately unused -- no code touches them.
 *
 * RULE THIS CLASS EXISTS UNDER: Panels may write the non-final fields (that is
 * what @Configurable is for). OpMode and mechanism code may only READ them.
 * Static fields outlive a single OpMode run, so a runtime write during run N
 * silently leaks into run N+1 -- which passes clean in any single test and
 * shows up later as "the robot behaved differently in match 2 for no reason".
 */
@Configurable
public class RobotConstants {

    // ---- Hardware names, confirmed against V2's robotConfigs.java ----
    public static final String SHOOTER_LEFT_NAME  = "ls";
    public static final String SHOOTER_RIGHT_NAME = "rs";
    public static final String INTAKE_NAME        = "intake";
    public static final String LEFT_GATE_NAME     = "lg";
    public static final String RIGHT_GATE_NAME    = "rg";
    public static final String TURRET_YAW_NAME    = "lt";
    public static final String TURRET_PITCH_NAME  = "panel";

    // =====================================================================
    // TARGET
    // =====================================================================
    /** Blue GOAL, V2 robotConstants.BLUE_TARGET_X/Y. Note it is (136, 136). */
    public static double BLUE_TARGET_X = 136.0;
    public static double BLUE_TARGET_Y = 136.0;

    // =====================================================================
    // SHOOTER -- V2's real distance model
    // =====================================================================
    // V2 fits velocity and hood as cubics in target distance (inches):
    //     f(x) = A*x^3 + B*x^2 + C*x + D
    // These replace the single hard-coded velocity used before, so the shooter
    // is correct at ANY distance rather than only at one.
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

    /** V2 panelConstants. Note KP 80 / KF 11 -- far stiffer than the 40/9.5 used before. */
    public static double SHOOTER_KP = 80.0;
    public static double SHOOTER_KI = 0.0;
    public static double SHOOTER_KD = 0.0;
    public static double SHOOTER_KF = 11.0;
    /** V2 VELOCITY_TOR. Ticks/sec. */
    public static double SHOOTER_VELOCITY_TOLERANCE = 20.0;
    /** V2 shooterHold() idle speed -- keeps the flywheel warm between shots. */
    public static double SHOOTER_HOLD_VELOCITY = 1300.0;

    // =====================================================================
    // GATES -- V2's measured servo positions
    // =====================================================================
    public static double LEFT_GATE_OPEN   = 0.431;
    public static double RIGHT_GATE_OPEN  = 0.582;
    public static double LEFT_GATE_CLOSE  = 0.616;
    public static double RIGHT_GATE_CLOSE = 0.390;

    // =====================================================================
    // INTAKE -- V2's powers
    // =====================================================================
    public static double INTAKE_POWER = 0.8;
    public static double INTAKE_SLOW_POWER = 0.35;
    /**
     * Feed power while shooting. V2 computes this from distance; this is the
     * clamp floor V2 uses, which is the safe constant choice for a fixed-distance
     * shot. Raise toward 1.0 if artifacts feed too slowly to make the volley.
     */
    public static double INTAKE_FIRE_POWER = 0.9;

    // =====================================================================
    // TURRET -- V2's real geometry. THIS IS THE BIGGEST CORRECTION.
    // =====================================================================
    /**
     * V2: 360 degrees over 1229 encoder ticks.
     *
     * The values used here previously were 180 deg / 668 ticks, borrowed from
     * another team. That is 0.2694 deg/tick against the true 0.2929 -- every
     * commanded turret angle was landing about 8.7% short, silently, while
     * telemetry reported the angle that had been asked for.
     */
    public static double TURRET_FULL_RANGE_DEGREE = 360.0;
    public static double TURRET_FULL_RANGE_ENCODER = 1229.0;
    public static double TURRET_YAW_TOLERANCE_DEG = 2.0;
    /** V2 drives the turret at full power under RUN_TO_POSITION with positional PIDF 20. */
    public static double TURRET_HOLD_POWER = 1.0;
    public static double TURRET_POSITION_PIDF_P = 20.0;

    /** Hood servo travel limits. V2 HOOD_LOWER/UPPER_LIMIT. */
    public static double PITCH_MIN = 0.0;
    public static double PITCH_MAX = 1.0;

    /**
     * Distance from drivetrain centre to the shooter, inches. V2:
     * SHOOTER_DRIVETRAIN_OFFSET = 61.5 mm / 25.4.
     */
    public static double SHOOTER_DRIVETRAIN_OFFSET = 61.5 / 25.4;

    // =====================================================================
    // Shot timing -- how long the gates stay open for one volley
    // =====================================================================
    public static double GATE_FEED_SECONDS = 0.9;

    /**
     * Assumed shooting distance for TeleOp, inches. TeleOp has no localizer of
     * its own, so it cannot solve distance the way the auto does -- the driver
     * dials this in from Panels instead. 136 in matches the auto's shooting
     * stops, so the same shot works out of the box.
     */
    public static double TELEOP_SHOOT_DISTANCE = 136.0;

    // ---------------------------------------------------------------------
    // Distance model, ported verbatim from V2's Shooter.java so the auto and
    // any future auto-aim use ONE source of truth for "how fast, how high".
    // ---------------------------------------------------------------------

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
