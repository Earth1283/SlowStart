package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="move the fucking bot")
public class MoveTheBot extends LinearOpMode{
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    leftFront = hardwareMap.get(DcMotor.class, "lf");
    leftBack = hardwareMap.get(DcMotor.class, "lb");
    rightFront = hardwareMap.get(DcMotor.class, "rf");
    rightFront = hardwareMap.get(DcMotor.class, "lb");

    public void handleController(){
        if (gamepad1.)
    }

    @Override
    public void runOpMode() {

    }
}
