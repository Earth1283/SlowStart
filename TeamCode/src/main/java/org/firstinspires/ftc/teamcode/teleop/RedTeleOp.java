package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kernel.constants.robotConstants;

/**
 * RED alliance driver-controlled period. All behaviour is in {@link AutoAimTeleOp};
 * this only picks the goal.
 *
 * Same PINPOINT -> PEDRO conversion as {@link BlueTeleOp}: pedroX = 144 - pinY,
 * pedroY = pinX. Kernel RED_TARGET (136, 8) becomes Pedro (136, 136).
 *
 * Cross-checked against 32008's own verified red auto: from their RED_FAR_SHOOT
 * (85, 17) at heading 0, this goal position gives a +67.79 deg turret solve
 * against their tuned +70.
 */
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
