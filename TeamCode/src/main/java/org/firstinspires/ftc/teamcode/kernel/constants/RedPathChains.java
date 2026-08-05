package org.firstinspires.ftc.teamcode.kernel.constants;

import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_CLOSE_INTAKE_SECOND_CONTROL;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_CLOSE_INTAKE_SECOND_END;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_CLOSE_SHOOT_PRELOAD;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_CLOSE_SHOOT_SECOND_ROW;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_CLOSE_INTAKE_FIRST_CONTROL;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_CLOSE_INTAKE_FIRST_END;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_CLOSE_INTAKE_GATE;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_CLOSE_INTAKE_THIRD_END;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_CLOSE_INTAKE_THIRD_START;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_CLOSE_SHOOT_CONTROL;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_CLOSE_SHOOT_FIRST_ROW;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_CLOSE_SHOOT_INTAKE_GATE;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_CLOSE_SHOOT_THIRD_ROW;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_FAR_INTAKE_HP_1_END;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_FAR_INTAKE_HP_1_MID;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_FAR_INTAKE_HP_1_START;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_FAR_INTAKE_THIRD_CONTROL;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_FAR_INTAKE_THIRD_END;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_FAR_INTAKE_TUNNEL_END;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_FAR_INTAKE_TUNNEL_START;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_FAR_SHOOT_HP;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_FAR_SHOOT_TUNNEL;
import static org.firstinspires.ftc.teamcode.kernel.constants.autoConstants.RED_FAR_START;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedPathChains {
    PathChain FAR_INTAKE_THIRD_ROW_CHAIN;
    PathChain FAR_INTAKE_HP_1_CHAIN;
    PathChain FAR_INTAKE_TUNNEL_CHAIN;
    PathChain CLOSE_INTAKE_FIRST_ROW_CHAIN;
    PathChain CLOSE_INTAKE_SECOND_ROW_CHAIN;
    PathChain CLOSE_INTAKE_THIRD_ROW_CHAIN;
    PathChain CLOSE_INTAKE_GATE_CHAIN;

    public RedPathChains(Follower follower) {
        FAR_INTAKE_THIRD_ROW_CHAIN = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(RED_FAR_START.getX(), RED_FAR_START.getY()),
                                new Pose(RED_FAR_INTAKE_THIRD_CONTROL.getX(), RED_FAR_INTAKE_THIRD_CONTROL.getY()),
                                new Pose(RED_FAR_INTAKE_THIRD_END.getX(), RED_FAR_INTAKE_THIRD_END.getY())
                        )
                )
                .setLinearHeadingInterpolation(RED_FAR_START.getHeading(), RED_FAR_INTAKE_THIRD_END.getHeading())
                .addPath(
                        new BezierLine(
                                new Pose(RED_FAR_INTAKE_THIRD_END.getX(), RED_FAR_INTAKE_THIRD_END.getY()),
                                new Pose(RED_FAR_SHOOT_TUNNEL.getX(), RED_FAR_SHOOT_TUNNEL.getY())
                        )
                )
                .setLinearHeadingInterpolation(RED_FAR_INTAKE_THIRD_END.getHeading(), RED_FAR_SHOOT_TUNNEL.getHeading())
                .build();

        FAR_INTAKE_HP_1_CHAIN = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(RED_FAR_SHOOT_HP.getX(), RED_FAR_SHOOT_HP.getY()),
                                new Pose(RED_FAR_INTAKE_HP_1_START.getX(), RED_FAR_INTAKE_HP_1_START.getY())
                        )
                )
                .setLinearHeadingInterpolation(RED_FAR_SHOOT_HP.getHeading(), RED_FAR_INTAKE_HP_1_START.getHeading())
                .addPath(
                        new BezierCurve(
                                new Pose(RED_FAR_INTAKE_HP_1_START.getX(), RED_FAR_INTAKE_HP_1_START.getY()),
                                new Pose(RED_FAR_INTAKE_HP_1_MID.getX(), RED_FAR_INTAKE_HP_1_MID.getY()),
                                new Pose(RED_FAR_INTAKE_HP_1_END.getX(), RED_FAR_INTAKE_HP_1_END.getY())
                        )
                )
                .setLinearHeadingInterpolation(RED_FAR_INTAKE_HP_1_START.getHeading(), RED_FAR_INTAKE_HP_1_END.getHeading())
                .build();

        FAR_INTAKE_TUNNEL_CHAIN = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(RED_FAR_SHOOT_TUNNEL.getX(), RED_FAR_SHOOT_TUNNEL.getY()),
                                new Pose(RED_FAR_INTAKE_TUNNEL_START.getX(), RED_FAR_INTAKE_TUNNEL_START.getY())
                        )
                )
                .setLinearHeadingInterpolation(RED_FAR_SHOOT_TUNNEL.getHeading(), RED_FAR_INTAKE_TUNNEL_START.getHeading())
                .addPath(
                        new BezierLine(
                                new Pose(RED_FAR_INTAKE_TUNNEL_START.getX(), RED_FAR_INTAKE_TUNNEL_START.getY()),
                                new Pose(RED_FAR_INTAKE_TUNNEL_END.getX(), RED_FAR_INTAKE_TUNNEL_END.getY())
                        )
                )
                .setLinearHeadingInterpolation(RED_FAR_INTAKE_TUNNEL_START.getHeading(), RED_FAR_INTAKE_TUNNEL_END.getHeading())
                .build();

        CLOSE_INTAKE_FIRST_ROW_CHAIN = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(RED_CLOSE_SHOOT_INTAKE_GATE.getX(), RED_CLOSE_SHOOT_INTAKE_GATE.getY()),
                                new Pose(RED_CLOSE_INTAKE_FIRST_CONTROL.getX(), RED_CLOSE_INTAKE_FIRST_CONTROL.getY()),
                                new Pose(RED_CLOSE_INTAKE_FIRST_END.getX(), RED_CLOSE_INTAKE_FIRST_END.getY())
                        )
                )
                .setLinearHeadingInterpolation(RED_CLOSE_SHOOT_INTAKE_GATE.getHeading(), RED_CLOSE_INTAKE_FIRST_END.getHeading())
                .addPath(
                        new BezierLine(
                                new Pose(RED_CLOSE_INTAKE_FIRST_END.getX(), RED_CLOSE_INTAKE_FIRST_END.getY()),
                                new Pose(RED_CLOSE_SHOOT_FIRST_ROW.getX(), RED_CLOSE_SHOOT_FIRST_ROW.getY())
                        )
                )
                .setLinearHeadingInterpolation(RED_CLOSE_INTAKE_FIRST_END.getHeading(), RED_CLOSE_SHOOT_FIRST_ROW.getHeading())
                .build();

        CLOSE_INTAKE_SECOND_ROW_CHAIN = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(RED_CLOSE_SHOOT_PRELOAD.getX(), RED_CLOSE_SHOOT_PRELOAD.getY()),
                                new Pose(RED_CLOSE_INTAKE_SECOND_CONTROL.getX(), RED_CLOSE_INTAKE_SECOND_CONTROL.getY()),
                                new Pose(RED_CLOSE_INTAKE_SECOND_END.getX(), RED_CLOSE_INTAKE_SECOND_END.getY())
                        )
                )
                .setLinearHeadingInterpolation(RED_CLOSE_SHOOT_PRELOAD.getHeading(), RED_CLOSE_INTAKE_SECOND_END.getHeading())
                .build();

        CLOSE_INTAKE_THIRD_ROW_CHAIN = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(RED_CLOSE_SHOOT_SECOND_ROW.getX(), RED_CLOSE_SHOOT_SECOND_ROW.getY()),
                                new Pose(RED_CLOSE_INTAKE_THIRD_START.getX(), RED_CLOSE_INTAKE_THIRD_START.getY())
                        )
                )
                .setLinearHeadingInterpolation(RED_CLOSE_SHOOT_SECOND_ROW.getHeading(), RED_CLOSE_INTAKE_THIRD_START.getHeading())
                .addPath(
                        new BezierLine(
                                new Pose(RED_CLOSE_INTAKE_THIRD_START.getX(), RED_CLOSE_INTAKE_THIRD_START.getY()),
                                new Pose(RED_CLOSE_INTAKE_THIRD_END.getX(), RED_CLOSE_INTAKE_THIRD_END.getY())
                        )
                )
                .setLinearHeadingInterpolation(RED_CLOSE_INTAKE_THIRD_START.getHeading(), RED_CLOSE_INTAKE_THIRD_END.getHeading())
                .addPath(
                        new BezierLine(
                                new Pose(RED_CLOSE_INTAKE_THIRD_END.getX(), RED_CLOSE_INTAKE_THIRD_END.getY()),
                                new Pose(RED_CLOSE_SHOOT_THIRD_ROW.getX(), RED_CLOSE_SHOOT_THIRD_ROW.getY())
                        )
                )
                .setLinearHeadingInterpolation(RED_CLOSE_INTAKE_THIRD_END.getHeading(), RED_CLOSE_SHOOT_THIRD_ROW.getHeading())
                .build();

        CLOSE_INTAKE_GATE_CHAIN = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(RED_CLOSE_SHOOT_INTAKE_GATE.getX(), RED_CLOSE_SHOOT_INTAKE_GATE.getY()),
                                new Pose(RED_CLOSE_SHOOT_CONTROL.getX(), RED_CLOSE_SHOOT_CONTROL.getY()),
                                new Pose(RED_CLOSE_INTAKE_GATE.getX(), RED_CLOSE_INTAKE_GATE.getY())
                        )
                )
                .setLinearHeadingInterpolation(RED_CLOSE_SHOOT_INTAKE_GATE.getHeading(), RED_CLOSE_INTAKE_GATE.getHeading())
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