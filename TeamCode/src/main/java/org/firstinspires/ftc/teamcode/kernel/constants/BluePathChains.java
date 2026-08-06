package org.firstinspires.ftc.teamcode.kernel.constants;

import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_CLOSE_INTAKE_FIRST_CONTROL;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_CLOSE_INTAKE_FIRST_END;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_CLOSE_INTAKE_GATE;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_CLOSE_INTAKE_SECOND_CONTROL;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_CLOSE_INTAKE_SECOND_END;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_CLOSE_INTAKE_THIRD_END;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_CLOSE_INTAKE_THIRD_START;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_CLOSE_SHOOT_CONTROL;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_CLOSE_SHOOT_FIRST_ROW;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_CLOSE_SHOOT_INTAKE_GATE;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_CLOSE_SHOOT_PRELOAD;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_CLOSE_SHOOT_SECOND_ROW;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_CLOSE_SHOOT_THIRD_ROW;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_FAR_INTAKE_HP_1_END;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_FAR_INTAKE_HP_1_MID;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_FAR_INTAKE_HP_1_START;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_FAR_INTAKE_THIRD_CONTROL;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_FAR_INTAKE_THIRD_END;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_FAR_INTAKE_TUNNEL_END;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_FAR_INTAKE_TUNNEL_START;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_FAR_SHOOT_HP;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_FAR_SHOOT_TUNNEL;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_FAR_START;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BluePathChains {
    PathChain FAR_INTAKE_THIRD_ROW_CHAIN;
    PathChain FAR_INTAKE_HP_1_CHAIN;
    PathChain FAR_INTAKE_TUNNEL_CHAIN;
    PathChain CLOSE_INTAKE_FIRST_ROW_CHAIN;
    PathChain CLOSE_INTAKE_SECOND_ROW_CHAIN;
    PathChain CLOSE_INTAKE_THIRD_ROW_CHAIN;
    PathChain CLOSE_INTAKE_GATE_CHAIN;

    public BluePathChains(Follower follower) {
        FAR_INTAKE_THIRD_ROW_CHAIN = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(BLUE_FAR_START.getX(), BLUE_FAR_START.getY()),
                                new Pose(BLUE_FAR_INTAKE_THIRD_CONTROL.getX(), BLUE_FAR_INTAKE_THIRD_CONTROL.getY()),
                                new Pose(BLUE_FAR_INTAKE_THIRD_END.getX(), BLUE_FAR_INTAKE_THIRD_END.getY())
                        )
                )
                .setLinearHeadingInterpolation(BLUE_FAR_START.getHeading(), BLUE_FAR_INTAKE_THIRD_END.getHeading())
                .addPath(
                        new BezierLine(
                                new Pose(BLUE_FAR_INTAKE_THIRD_END.getX(), BLUE_FAR_INTAKE_THIRD_END.getY()),
                                new Pose(BLUE_FAR_SHOOT_TUNNEL.getX(), BLUE_FAR_SHOOT_TUNNEL.getY())
                        )
                )
                .setLinearHeadingInterpolation(BLUE_FAR_INTAKE_THIRD_END.getHeading(), BLUE_FAR_SHOOT_TUNNEL.getHeading())
                .build();

        FAR_INTAKE_HP_1_CHAIN = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(BLUE_FAR_SHOOT_HP.getX(), BLUE_FAR_SHOOT_HP.getY()),
                                new Pose(BLUE_FAR_INTAKE_HP_1_START.getX(), BLUE_FAR_INTAKE_HP_1_START.getY())
                        )
                )
                .setLinearHeadingInterpolation(BLUE_FAR_SHOOT_HP.getHeading(), BLUE_FAR_INTAKE_HP_1_START.getHeading())
                .addPath(
                        new BezierCurve(
                                new Pose(BLUE_FAR_INTAKE_HP_1_START.getX(), BLUE_FAR_INTAKE_HP_1_START.getY()),
                                new Pose(BLUE_FAR_INTAKE_HP_1_MID.getX(), BLUE_FAR_INTAKE_HP_1_MID.getY()),
                                new Pose(BLUE_FAR_INTAKE_HP_1_END.getX(), BLUE_FAR_INTAKE_HP_1_END.getY())
                        )
                )
                .setLinearHeadingInterpolation(BLUE_FAR_INTAKE_HP_1_START.getHeading(), BLUE_FAR_INTAKE_HP_1_END.getHeading())
                .build();

        FAR_INTAKE_TUNNEL_CHAIN = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(BLUE_FAR_SHOOT_TUNNEL.getX(), BLUE_FAR_SHOOT_TUNNEL.getY()),
                                new Pose(BLUE_FAR_INTAKE_TUNNEL_START.getX(), BLUE_FAR_INTAKE_TUNNEL_START.getY())
                        )
                )
                .setLinearHeadingInterpolation(BLUE_FAR_SHOOT_TUNNEL.getHeading(), BLUE_FAR_INTAKE_TUNNEL_START.getHeading())
                .addPath(
                        new BezierLine(
                                new Pose(BLUE_FAR_INTAKE_TUNNEL_START.getX(), BLUE_FAR_INTAKE_TUNNEL_START.getY()),
                                new Pose(BLUE_FAR_INTAKE_TUNNEL_END.getX(), BLUE_FAR_INTAKE_TUNNEL_END.getY())
                        )
                )
                .setLinearHeadingInterpolation(BLUE_FAR_INTAKE_TUNNEL_START.getHeading(), BLUE_FAR_INTAKE_TUNNEL_END.getHeading())
                .build();

        CLOSE_INTAKE_FIRST_ROW_CHAIN = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(BLUE_CLOSE_SHOOT_PRELOAD.getX(), BLUE_CLOSE_SHOOT_PRELOAD.getY()),
                                new Pose(BLUE_CLOSE_INTAKE_FIRST_CONTROL.getX(), BLUE_CLOSE_INTAKE_FIRST_CONTROL.getY()),
                                new Pose(BLUE_CLOSE_INTAKE_FIRST_END.getX(), BLUE_CLOSE_INTAKE_FIRST_END.getY())
                        )
                )
                .setLinearHeadingInterpolation(BLUE_CLOSE_SHOOT_PRELOAD.getHeading(), BLUE_CLOSE_INTAKE_FIRST_END.getHeading())
                .addPath(
                        new BezierLine(
                                new Pose(BLUE_CLOSE_INTAKE_FIRST_END.getX(), BLUE_CLOSE_INTAKE_FIRST_END.getY()),
                                new Pose(BLUE_CLOSE_SHOOT_FIRST_ROW.getX(), BLUE_CLOSE_SHOOT_FIRST_ROW.getY())
                        )
                )
                .setLinearHeadingInterpolation(BLUE_CLOSE_INTAKE_FIRST_END.getHeading(), BLUE_CLOSE_SHOOT_FIRST_ROW.getHeading())
                .build();

        CLOSE_INTAKE_SECOND_ROW_CHAIN = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(BLUE_CLOSE_SHOOT_PRELOAD.getX(), BLUE_CLOSE_SHOOT_PRELOAD.getY()),
                                new Pose(BLUE_CLOSE_INTAKE_SECOND_CONTROL.getX(), BLUE_CLOSE_INTAKE_SECOND_CONTROL.getY()),
                                new Pose(BLUE_CLOSE_INTAKE_SECOND_END.getX(), BLUE_CLOSE_INTAKE_SECOND_END.getY())
                        )
                )
                .setLinearHeadingInterpolation(BLUE_CLOSE_SHOOT_PRELOAD.getHeading(), BLUE_CLOSE_INTAKE_SECOND_END.getHeading())

                .build();

        CLOSE_INTAKE_THIRD_ROW_CHAIN = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(BLUE_CLOSE_SHOOT_SECOND_ROW.getX(), BLUE_CLOSE_SHOOT_SECOND_ROW.getY()),
                                new Pose(BLUE_CLOSE_INTAKE_THIRD_START.getX(), BLUE_CLOSE_INTAKE_THIRD_START.getY())
                        )
                )
                .setLinearHeadingInterpolation(BLUE_CLOSE_SHOOT_SECOND_ROW.getHeading(), BLUE_CLOSE_INTAKE_THIRD_START.getHeading())
                .addPath(
                        new BezierLine(
                                new Pose(BLUE_CLOSE_INTAKE_THIRD_START.getX(), BLUE_CLOSE_INTAKE_THIRD_START.getY()),
                                new Pose(BLUE_CLOSE_INTAKE_THIRD_END.getX(), BLUE_CLOSE_INTAKE_THIRD_END.getY())
                        )
                )
                .setLinearHeadingInterpolation(BLUE_CLOSE_INTAKE_THIRD_START.getHeading(), BLUE_CLOSE_INTAKE_THIRD_END.getHeading())
                .addPath(
                        new BezierLine(
                                new Pose(BLUE_CLOSE_INTAKE_THIRD_END.getX(), BLUE_CLOSE_INTAKE_THIRD_END.getY()),
                                new Pose(BLUE_CLOSE_SHOOT_THIRD_ROW.getX(), BLUE_CLOSE_SHOOT_THIRD_ROW.getY())
                        )
                )
                .setLinearHeadingInterpolation(BLUE_CLOSE_INTAKE_THIRD_END.getHeading(), BLUE_CLOSE_SHOOT_THIRD_ROW.getHeading())
                .build();

        CLOSE_INTAKE_GATE_CHAIN = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(BLUE_CLOSE_SHOOT_INTAKE_GATE.getX(), BLUE_CLOSE_SHOOT_INTAKE_GATE.getY()),
                                new Pose(BLUE_CLOSE_SHOOT_CONTROL.getX(), BLUE_CLOSE_SHOOT_CONTROL.getY()),
                                new Pose(BLUE_CLOSE_INTAKE_GATE.getX(), BLUE_CLOSE_INTAKE_GATE.getY())
                        )
                )
                .setLinearHeadingInterpolation(BLUE_CLOSE_SHOOT_INTAKE_GATE.getHeading(), BLUE_CLOSE_INTAKE_GATE.getHeading())
                .build();
    }

    public PathChain getFAR_INTAKE_THIRD_ROW_CHAIN() {
        return FAR_INTAKE_THIRD_ROW_CHAIN;
    }

    public PathChain getFAR_INTAKE_HP_1_CHAIN() {
        return FAR_INTAKE_HP_1_CHAIN;
    }

    public PathChain getFAR_INTAKE_TUNNEL_CHAIN() {
        return FAR_INTAKE_TUNNEL_CHAIN;
    }

    public PathChain getCLOSE_INTAKE_FIRST_ROW_CHAIN() {
        return CLOSE_INTAKE_FIRST_ROW_CHAIN;
    }

    public PathChain getCLOSE_INTAKE_SECOND_ROW_CHAIN() {
        return CLOSE_INTAKE_SECOND_ROW_CHAIN;
    }

    public PathChain getCLOSE_INTAKE_THIRD_ROW_CHAIN() {
        return CLOSE_INTAKE_THIRD_ROW_CHAIN;
    }

    public PathChain getCLOSE_INTAKE_GATE_CHAIN() {
        return CLOSE_INTAKE_GATE_CHAIN;
    }
}
