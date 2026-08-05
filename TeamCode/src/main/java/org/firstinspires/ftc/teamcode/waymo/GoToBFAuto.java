package org.firstinspires.ftc.teamcode.waymo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kernel.motion.GoTo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Navigate to BlueFarAuto")
public class GoToBFAuto extends LinearOpMode {

    private static final Pose START_POSE  = new Pose(8.0, 8.0, Math.toRadians(90));
    private static final Pose TARGET_POSE = new Pose(56.0, 8.0, Math.toRadians(90));

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        GoTo goTo = new GoTo(follower);

        waitForStart();

        goTo.goTo(START_POSE, TARGET_POSE);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }
}
