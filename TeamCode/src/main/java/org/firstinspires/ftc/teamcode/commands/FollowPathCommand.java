package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;

/**
 * Team 32008's own FollowPathCommand, copied from FTC-32008 V2 commands/FollowPathCommand.java.
 *
 * ONLY changes: the constants imports point at this repo's kernel.constants,
 * whose classes were renamed to PascalCase (autoConstants -> AutoConstants and
 * so on). No logic touched.
 *
 * NOTE it resolves pedroPathing.Constants to THIS repo's tuned copy, not V2's --
 * that is deliberate: those are the constants measured on this robot.
 */
public class FollowPathCommand extends CommandBase {
    private final FollowerSubsystem follower;
    private final PathChain pathChain;
    private boolean holdEnd;
    private double maxPower = 1.0;

    public FollowPathCommand(FollowerSubsystem follower, PathChain pathChain) {
        this(follower, pathChain, true);
    }

    public FollowPathCommand(FollowerSubsystem follower, PathChain pathChain, boolean holdEnd) {
        this(follower, pathChain, holdEnd, 1.0);
    }

    public FollowPathCommand(FollowerSubsystem follower, PathChain pathChain, double maxPower) {
        this(follower, pathChain, true, maxPower);
    }

    public FollowPathCommand(FollowerSubsystem follower, PathChain pathChain, boolean holdEnd, double maxPower) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    public FollowPathCommand(FollowerSubsystem follower, Path path) {
        this(follower, path, true);
    }

    public FollowPathCommand(FollowerSubsystem follower, Path path, boolean holdEnd) {
        this(follower, path, holdEnd, 1.0);
    }

    public FollowPathCommand(FollowerSubsystem follower, Path path, double maxPower) {
        this(follower, path, true, maxPower);
    }

    public FollowPathCommand(FollowerSubsystem follower, Path path, boolean holdEnd, double maxPower) {
        this.follower = follower;
        this.pathChain = new PathChain(path);
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    public FollowPathCommand setGlobalMaxPower(double globalMaxPower) {
        follower.follower.setMaxPower(globalMaxPower);
        maxPower = globalMaxPower;
        return this;
    }

    @Override
    public void initialize() {
        follower.follower.followPath(pathChain, maxPower, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy() || follower.isRobotStuck() || follower.follower.atParametricEnd();
    }

    @Override
    public void end(boolean interrupted){
    }
}