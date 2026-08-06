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
import org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs;

public class Constants {

    // WARNING: PedroPathing constants were found by running the Tuning script.
    // These are as good as magic numbers, so don't change unless you changed something in the bot's
    // physical layout

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.7)
            .forwardZeroPowerAcceleration(-27.347306480528307)
            .lateralZeroPowerAcceleration(-56.35896982484836)
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryTranslationalPIDF(false)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.013, 0.0, 0.0005, 0.6, 0.0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0.0, 0.025, 0.0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.1, 0.0, 0.11, 0.0))
            .centripetalScaling(0.0005)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(
                    0.1, 0.06444429384663601, 0.0023836848323013143))
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName(RobotConfigs.LEFT_FRONT)
            .leftRearMotorName(RobotConfigs.LEFT_BACK)
            .rightFrontMotorName(RobotConfigs.RIGHT_FRONT)
            .rightRearMotorName(RobotConfigs.RIGHT_BACK)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(81.18051892378199)
            .yVelocity(64.14246230238066)
            .useVoltageCompensation(true)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-141.5)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName(RobotConfigs.PIN_POINT)
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
