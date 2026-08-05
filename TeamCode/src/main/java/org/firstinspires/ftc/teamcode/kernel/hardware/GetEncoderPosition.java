package org.firstinspires.ftc.teamcode.kernel.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class GetEncoderPosition {
    /**
     * Gets the current encoder in ticks for a given DcMotor
     * @param motor - A motor, expects DcMotorEx
     * @return - Returns the encoder, ticks, as an integer
     */
    public static int getEncoderPosition(DcMotorEx motor) {
        return motor.getCurrentPosition();
    }
}
