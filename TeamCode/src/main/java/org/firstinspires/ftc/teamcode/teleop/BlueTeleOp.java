package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kernel.constants.robotConstants;

/**
 * BLUE alliance driver-controlled period. All behaviour is in {@link AutoAimTeleOp};
 * this only picks the goal.
 *
 * The kernel states goals in the PINPOINT frame (pinX = pedroY,
 * pinY = 144 - pedroX, per V2 tests/AASSTEST.java:59). Converted to Pedro here,
 * which is the frame the follower and the auto both work in.
 *
 * BLUE_GOAL_X/Y must read 8 / 136. If they read 136 / 136 that is the RED goal
 * and the turret will aim across the field.
 */
@TeleOp(name = "32008 BLUE TeleOp", group = "32008")
@Configurable
public class BlueTeleOp extends AutoAimTeleOp {

    public static double BLUE_GOAL_X = 144.0 - robotConstants.BLUE_TARGET_Y;
    public static double BLUE_GOAL_Y = robotConstants.BLUE_TARGET_X;

    @Override
    protected double goalX() {
        return BLUE_GOAL_X;
    }

    @Override
    protected double goalY() {
        return BLUE_GOAL_Y;
    }

    @Override
    protected String allianceName() {
        return "BLUE";
    }
}
