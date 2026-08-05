package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing configuration for FTC team 32008 (DECODE 2025-26).
 *
 * ============================================================================
 * SOURCE: adopted WHOLESALE from 32008's competition-verified "FTC-32008 V2"
 * repository, pedroPathing/Constants.java. These are this team's own numbers
 * from their own working robot -- not borrowed from another team, not derived.
 *
 * Adopted as a COMPLETE SET on purpose. A PIDF/mass/acceleration model is tuned
 * together; mixing values from two different tuning sessions is how a robot ends
 * up behaving in a way nobody can reproduce or explain. Where a value measured
 * in this workspace differs, the alternative is recorded in the comment beside
 * it so it can be A/B tested deliberately -- but do not swap them one at a time.
 * ============================================================================
 */
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            // V2: 13.7 kg. Earlier measurement in this workspace was 12.55 -- if the
            // robot genuinely weighs 12.55 now, the whole V2 set was tuned at 13.7 and
            // should be re-checked rather than having mass alone edited.
            .mass(13.7)
            // V2 verified. Workspace tuner runs gave -23.694 / -59.226 -- close on
            // lateral, 15% off on forward.
            .forwardZeroPowerAcceleration(-27.347306480528307)
            .lateralZeroPowerAcceleration(-56.35896982484836)
            // V2 states the single-PIDF choice explicitly rather than relying on
            // library defaults. Kept, because it documents intent.
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryTranslationalPIDF(false)
            // FilteredPIDFCoefficients order is (P, I, D, T, F) -- T is the filter in
            // slot 4, F in slot 5. The field DECLARATION order in that class is
            // P,I,D,F,T, which is different. Do not reorder by reading the field list.
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.013, 0.0, 0.0005, 0.6, 0.0))
            // V2: P 0.15. Workspace Translational Tuner gave P 0.11, same D.
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0.0, 0.025, 0.0))
            // V2 verified. This replaces an UNVERIFIED estimate of (1.5, 0, 0.32) that
            // was reasoned from a wobble report -- the real robot runs much lower P and
            // much lower D. That estimate was the least-grounded value in this project.
            .headingPIDFCoefficients(new PIDFCoefficients(1.1, 0.0, 0.11, 0.0))
            // V2 runs centripetal ENABLED at Pedro's default. It was previously disabled
            // (0) here. Harmless on the all-straight-line path below, but correct to
            // have on the moment any BezierCurve is added.
            .centripetalScaling(0.0005)
            // V2 verified. Order is (P, kLinearBraking, kQuadraticFriction) -- P first,
            // verified by decompiling PredictiveBrakingCoefficients in core-2.1.2.
            // Workspace brake tuner gave (0.15, 0.09176, 0.001675).
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(
                    0.1, 0.06444429384663601, 0.0023836848323013143))
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // V2: 81.18 / 64.14. Workspace tuners gave 80.85 / 63.45 -- agreement to
            // within 0.5%, which is strong independent confirmation that both the
            // drivetrain and the localizer are set up correctly.
            .xVelocity(81.18051892378199)
            .yVelocity(64.14246230238066)
            .useVoltageCompensation(true)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            // V2 works in MILLIMETRES. -141.5 mm = -5.571 in. The value previously used
            // here was -5.3150 in (-135 mm) from a hand measurement -- 6.5 mm out.
            .forwardPodY(-141.5)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.MM)
            // Pinpoint on Control Hub I2C bus 0. Pedro's docs warn against bus 0 because
            // the Control Hub's internal IMU lives there, but .pinpointLocalizer never
            // opens that IMU, so nothing contends for the bus. That stops being true the
            // moment any code instantiates the built-in IMU.
            .hardwareMapName("pp")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            ;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
