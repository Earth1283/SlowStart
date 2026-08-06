package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants;

@TeleOp(name = "32008 BLUE TeleOp", group = "32008")
@Configurable
public class BlueTeleOp extends AutoAimTeleOp {

    public static double BLUE_GOAL_X = 144.0 - RobotConstants.BLUE_TARGET_Y;
    public static double BLUE_GOAL_Y = RobotConstants.BLUE_TARGET_X;

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
