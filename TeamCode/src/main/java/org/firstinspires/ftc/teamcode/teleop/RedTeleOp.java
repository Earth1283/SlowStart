package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants;

/**
 * RED alliance. All behaviour is in {@link AllianceTeleOp}; this only picks the goal.
 * Matches their A_1_AA_AS, which targets RED_TARGET_X/Y.
 *
 * PINPOINT frame -- kernel value used as stated, no conversion. See {@link BlueTeleOp}.
 */
@TeleOp(name = "32008 RED TeleOp", group = "32008")
@Configurable
public class RedTeleOp extends AllianceTeleOp {

    @Override
    protected double targetXForAlliance() {
        return RobotConstants.RED_TARGET_X;
    }

    @Override
    protected double targetYForAlliance() {
        return RobotConstants.RED_TARGET_Y;
    }

    @Override
    protected String allianceName() {
        return "RED";
    }
}
