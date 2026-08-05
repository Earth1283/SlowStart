package org.firstinspires.ftc.teamcode.kernel.motion;

import org.firstinspires.ftc.teamcode.kernel.safety.ValueChecks;

public class ComputeAim {
    /**
     * Find the angle delta between angle1 and angle2, automatically normalized
     * @param angle1 - the first angle
     * @param angle2 - the second angle
     * @return - returns the normalized delta angle
     */
    public static double findDelta(double angle1, double angle2) {
        double normalizedAngle1 = ValueChecks.normalizeAngle(angle1);
        double normalizedAngle2 = ValueChecks.normalizeAngle(angle2);

        return ValueChecks.normalizeAngle(normalizedAngle1 - normalizedAngle2);
    }
}
