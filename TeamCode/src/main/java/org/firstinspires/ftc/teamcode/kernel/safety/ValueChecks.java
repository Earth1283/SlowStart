package org.firstinspires.ftc.teamcode.kernel.safety;

public class ValueChecks {
    /*
    Pass an object into checkNotNullParameter, and it will
    return true if it is not null, false if not null
     */
    public boolean checkNullParameter(Object obj) {
        return obj != null;
    }

    public boolean checkInRange(double value, double min, double max) {
        return value >= min && value <= max;
    }

    public boolean checkMotorPower(double power) {
        return checkInRange(power, -1.0, 1.0);
    }

    public double clampMotorPower(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
    }

    public boolean checkFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    public boolean checkPositive(double value) {
        return value > 0;
    }

    public boolean checkNonNegative(double value) {
        return value >= 0;
    }

    public boolean checkNotEmpty(String value) {
        return value != null && !value.trim().isEmpty();
    }

    public boolean checkNotEmpty(Object[] array) {
        return array != null && array.length > 0;
    }

    public boolean checkServoPosition(double position) {
        return checkInRange(position, 0.0, 1.0);
    }

    public double clampServoPosition(double position) {
        return Math.max(0.0, Math.min(1.0, position));
    }

    public int normalizeAngle(int angle) {
        int normalized = angle % 360;
        normalized = (normalized + 360) % 360;
        if (normalized > 180) {
            normalized -= 360;
        }
        return normalized;
    }

    public double normalizeAngle(double angle) {
        double normalized = angle % 360;
        normalized = (normalized + 360) % 360; // Force to [0, 360)
        if (normalized > 180) {
            normalized -= 360;
        }
        return normalized;
    }
}
