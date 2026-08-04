package org.firstinspires.ftc.teamcode.kernel.exceptions;

public class UnsafeToContinue extends RuntimeException {
    public UnsafeToContinue(){
        super("I'm sorry Dave, I'm afraid I can't do that. This mission is too important for me to allow you to jeopardize it.");
    }
    public UnsafeToContinue(String message) {
        super(message);
    }
}
