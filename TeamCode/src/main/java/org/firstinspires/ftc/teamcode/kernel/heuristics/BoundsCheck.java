package org.firstinspires.ftc.teamcode.kernel.heuristics;

import org.firstinspires.ftc.teamcode.kernel.exceptions.UnsafeToContinue;

public class BoundsCheck {

    public double acceptableError = 2.5;
    public double leniency = 0.0;

    /*
    Is the robot outside the acceptable left range of
    (-acceptable, -acceptable)?
     */
    public void isInBounds(double x, double y, boolean lenient) {
        if (lenient) {
            leniency = 6.9;
        } else {
            leniency = 0.0;
        }
        if (x < - acceptableError - leniency || y < -acceptableError - leniency){
            throw new UnsafeToContinue("The odometry has drifted too far, please inspect");
        }
        if (x > 144 + acceptableError + leniency || y > 144 + acceptableError + leniency) {
            throw new UnsafeToContinue("The odometry has drifted too far, please inspect");
        }
    }
}
