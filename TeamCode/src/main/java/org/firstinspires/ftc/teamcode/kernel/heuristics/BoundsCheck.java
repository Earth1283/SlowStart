package org.firstinspires.ftc.teamcode.kernel.heuristics;

import org.firstinspires.ftc.teamcode.kernel.exceptions.InvalidCoordinates;

public class BoundsCheck {

    public static void isInBounds(double x, double y) throws InvalidCoordinates {
        if (x < 0 || y < 0){
            throw new InvalidCoordinates();
        }
        if (x > 144 || y > 144) {
            throw new InvalidCoordinates();
        }
    }
}
