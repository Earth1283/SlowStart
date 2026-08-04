package org.firstinspires.ftc.teamcode.kernel.exceptions;

import java.lang.RuntimeException;

public class UnsafeToContinue extends RuntimeException {
    public UnsafeToContinue(){
        super("It is currently unsafe to continue the operation of the robot. Reason unspecified.");
    }
    public UnsafeToContinue(String message) {
        super(message);
    }
}
