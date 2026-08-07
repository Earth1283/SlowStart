package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Team 32008's own FollowerSubsystem, copied from FTC-32008 V2 subsystems/FollowerSubsystem.java.
 *
 * ONLY changes: the constants imports point at this repo's kernel.constants,
 * whose classes were renamed to PascalCase (autoConstants -> AutoConstants and
 * so on). No logic touched.
 *
 * NOTE it resolves pedroPathing.Constants to THIS repo's tuned copy, not V2's --
 * that is deliberate: those are the constants measured on this robot.
 */
public class FollowerSubsystem extends SubsystemBase {
    private static FollowerSubsystem instance;
    private TelemetryManager telemetry;
    public  Follower follower;

    public FollowerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry){
        follower = Constants.createFollower(hardwareMap);
        this.telemetry = telemetry;
        register();
    }

    public static synchronized FollowerSubsystem getInstance(HardwareMap hardwareMap, TelemetryManager telemetry){
        instance = new FollowerSubsystem(hardwareMap, telemetry);
        return instance;
    }

    @Override
    public void periodic() {
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
    }

    public void followPath(Path path, boolean holdEnd){
        follower.followPath(path, holdEnd);
    }

    public void followPath(PathChain path, boolean holdEnd){
        follower.followPath(path, holdEnd);
    }

    public void setStartingPose(Pose a){
        follower.setStartingPose(a.copy());
    }

    public Pose getPose(){
        return follower.getPose();
    }

    public boolean isRobotStuck(){
        return follower.isRobotStuck();
    }

    public boolean isBusy(){
        return follower.isBusy();
    }

    public double getCurrentTValue(){
        return follower.getCurrentTValue();
    }

    public PathBuilder pathBuilder(){
        return follower.pathBuilder();
    }

    public void breakFollowing(){ follower.breakFollowing(); }

    public boolean getatParametricEnd(){
        return follower.atParametricEnd();
    }
}
