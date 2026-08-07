package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants;

/**
 * BLUE alliance. All behaviour is in {@link AllianceTeleOp}; this only picks the goal.
 *
 * PINPOINT frame, which is the frame that whole TeleOp works in -- so the kernel
 * value is used AS STATED, no conversion. (The auto works in the PEDRO frame and
 * does convert; the two must not be mixed.)
 */
@TeleOp(name = "32008 BLUE TeleOp", group = "32008")
@Configurable
public class BlueTeleOp extends AllianceTeleOp {

    @Override
    protected double targetXForAlliance() {
        return RobotConstants.BLUE_TARGET_X;
    }

    @Override
    protected double targetYForAlliance() {
        return RobotConstants.BLUE_TARGET_Y;
    }

    @Override
    protected String allianceName() {
        return "BLUE";
    }
}
