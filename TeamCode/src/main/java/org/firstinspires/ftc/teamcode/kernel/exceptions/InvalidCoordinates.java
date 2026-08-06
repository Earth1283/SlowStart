package org.firstinspires.ftc.teamcode.kernel.exceptions;

public class InvalidCoordinates extends Exception{
    public InvalidCoordinates(String message) {
        super(message);
    }
    public InvalidCoordinates() {
        super("The provided coordinates are not within the valid FTC Field");
    }
}
