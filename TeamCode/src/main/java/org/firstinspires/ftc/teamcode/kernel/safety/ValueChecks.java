package org.firstinspires.ftc.teamcode.kernel.safety;

public class ValueChecks {
    public static boolean isValueValid(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    public static boolean checkInRange(double value, double min, double max) {
        return value >= min && value <= max;
    }

    public static boolean checkNotEmpty(String value) {
        return value != null && !value.trim().isEmpty();
    }

    public static boolean checkNotEmpty(Object[] array) {
        return array != null && array.length > 0;
    }

    public static int normalizeAngle(int angle) {
        int normalized = angle % 360;
        normalized = (normalized + 360) % 360;
        if (normalized > 180) {
            normalized -= 360;
        }
        return normalized;
    }

    // quick overload of normalizeAngle to support doubles
    public static double normalizeAngle(double angle) {
        double normalized = angle % 360;
        normalized = (normalized + 360) % 360; // Force to [0, 360)
        if (normalized > 180) {
            normalized -= 360;
        }
        return normalized;
    }
}
