package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kernel.constants.robotConstants;

@TeleOp(name = "32008 RED TeleOp", group = "32008")
@Configurable
public class RedTeleOp extends AutoAimTeleOp {

    public static double RED_GOAL_X = 144.0 - robotConstants.RED_TARGET_Y;
    public static double RED_GOAL_Y = robotConstants.RED_TARGET_X;

    @Override
    protected double goalX() {
        return RED_GOAL_X;
    }

    @Override
    protected double goalY() {
        return RED_GOAL_Y;
    }

    @Override
    protected String allianceName() {
        return "RED";
    }
}
