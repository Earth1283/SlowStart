package org.firstinspires.ftc.teamcode.kernel.heuristics;

import org.firstinspires.ftc.teamcode.kernel.exceptions.UnsafeToContinue;

public class BoundsCheck {

    public double acceptableError = 2.5;

    /*
    Is the robot outside the acceptable left range of
    (-acceptable, -acceptable)?
     */
    public void isInBounds(int x, int y) {
        if (x < acceptableError || y < acceptableError){
            throw new UnsafeToContinue("The odometry has drifted too far, please inspect.");
        }
        if (x > 141.1 + acceptableError || y > 141.1 + acceptableError) {
            throw new UnsafeToContinue("The odometry has drifted too far, please inspect.");
        }
    }
}
