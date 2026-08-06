package org.firstinspires.ftc.teamcode.kernel.motion;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.kernel.exceptions.InvalidCoordinates;
import org.firstinspires.ftc.teamcode.kernel.heuristics.BoundsCheck;
import org.firstinspires.ftc.teamcode.kernel.safety.ValueChecks;

/**
 * Thin wrapper around a Pedro {@link Follower} for one-shot point-to-point motion.
 * Builds a single-segment {@link BezierLine} path with linear heading interpolation
 * between two poses and hands it to the follower to execute.
 */
public class GoTo {
    private final Follower follower;

    /**
     * @param follower the Pedro follower that will build and run paths
     */
    public GoTo(Follower follower) {
        this.follower = follower;
    }

    /**
     * Builds a straight-line path from currentPose to targetPose, linearly interpolating
     * heading over the segment, and starts the follower on it (holds end pose when done).
     *
     * @param currentPose starting pose of the path
     * @param targetPose  destination pose, including desired ending heading
     */
    public void goTo(Pose currentPose, Pose targetPose) {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .build();
        follower.followPath(path, true);
    }

    /**
     * Raw-double overload of {@link #goTo(Pose, Pose)}; wraps the coordinates into
     * {@link Pose} instances and delegates. Here to maintain legacy compatibility
     * Parameters are self-documenting.
     */
    public void goTo(double currentX, double currentY, double currentHdg,
                     double targetX, double targetY, double targetHdg) throws InvalidCoordinates {
        BoundsCheck.isInBounds(targetX, targetY);
        goTo(new Pose(currentX, currentY, ValueChecks.normalizeAngle(currentHdg)),
                new Pose(targetX, targetY, ValueChecks.normalizeAngle(targetHdg)));
    }
}
