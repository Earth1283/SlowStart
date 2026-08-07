package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConfigs.TURRET;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.TURRET_FULL_RANGE_DEGREE;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.TURRET_FULL_RANGE_ENCODER;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.AutoAimSubsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * Team 32008's own TurretFullAutoTuner, copied from FTC-32008 V2 tests/TurretFullAutoTuner.java.
 *
 * ONLY changes: the constants imports point at this repo's kernel.constants,
 * whose classes were renamed to PascalCase (autoConstants -> AutoConstants and
 * so on). No logic touched.
 *
 * NOTE it resolves pedroPathing.Constants to THIS repo's tuned copy, not V2's --
 * that is deliberate: those are the constants measured on this robot.
 */
@TeleOp(name = "Turret Full Auto Tuner", group = "Tuning")
@Configurable
public class TurretFullAutoTuner extends OpMode {
    private DcMotorEx turret;

    public static double MAX_SAFE_ANGLE = 170.0;
    public static double BRAKE_BUFFER_ANGLE = 12.0;
    public static double HARD_STOP_MARGIN_ANGLE = 8.0;
    public static double RETURN_POWER = 0.25;
    public static double RETURN_TOLERANCE = 3.0;
    public static int REST_TIME_MS = 500;

    public static double SIGN_CHECK_POWER = 0.1;
    public static int SIGN_CHECK_MS = 250;
    public static double SIGN_CHECK_MIN_DELTA_DEG = 0.5;

    public static double KS_POWER_RAMP_RATE_PER_SEC = 0.12;
    public static double MAX_TEST_POWER = 0.85;
    public static double[] KV_TEST_POWERS = {0.2, 0.3, 0.45, 0.6, 0.75};
    public static double[] KA_TEST_POWERS = {0.25, 0.4, 0.55, 0.7};
    public static double[] PB_TEST_POWERS = {0.75, 0.6, 0.45, 0.3};
    public static double PB_BRAKING_POWER = 0.25;
    public static int PB_SPIN_TIME_MS = 700;

    public static double VEL_FILTER_ALPHA = 0.8;
    public static double ACCEL_FILTER_ALPHA = 0.8;
    public static double REFERENCE_VOLTAGE = AutoAimSubsystem.TUNING_VOLTAGE;

    private enum State {
        SIGN_CHECK, SIGN_RETURN, SIGN_WAIT,
        START,
        KS_RUN, KS_RETURN, KS_WAIT,
        KV_RUN, KV_RETURN, KV_WAIT,
        KA_RUN, KA_RETURN, KA_WAIT,
        PB_RUN, PB_BRAKE, PB_RETURN, PB_WAIT,
        CALCULATE, DONE, ERROR
    }

    private State state = State.SIGN_CHECK;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private long lastTime = 0;
    private double lastAngle = 0.0;
    private double filteredVel = 0.0;
    private double lastFilteredVel = 0.0;
    private double filteredAccel = 0.0;
    private boolean isInitialized = false;

    private double currentVoltage = 12.0;
    private double sumVoltage = 0.0;
    private int voltageSamples = 0;

    private int iteration = 0;
    private int currentDirection = 1;
    private int powerSignForPositiveAngle = 1;
    private double signStartAngle = 0.0;
    private double signCheckDelta = 0.0;
    private double currentTestPower = 0.0;
    private double pbStartBrakeAngle = 0.0;
    private double pbMeasuredBrakeVel = 0.0;

    private final List<Double> ksSamples = new ArrayList<>();
    private final List<double[]> kvData = new ArrayList<>();
    private final List<double[]> kaData = new ArrayList<>();
    private final List<double[]> pbData = new ArrayList<>();

    private double rawKs = 0.0;
    private double rawKv = 0.0;
    private double rawKa = 0.0;
    private double rawKLin = 0.0;
    private double rawKQuad = 0.0;
    private double compKs = 0.0;
    private double compKv = 0.0;
    private double compKa = 0.0;
    private double compKLin = 0.0;
    private double compKQuad = 0.0;
    private double avgTestVoltage = 0.0;
    private String errorMessage = "";

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, TURRET);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready to auto-tune turret feedforward and braking.");
        telemetry.addData("Angle conversion", "%.3f ticks/deg", TURRET_FULL_RANGE_ENCODER / TURRET_FULL_RANGE_DEGREE);
        telemetry.addData("Safe angle", "+/- %.1f deg", MAX_SAFE_ANGLE);
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
        voltageTimer.reset();
        lastTime = System.nanoTime();
        signStartAngle = getCurrentAngleDeg();
        resetMotionFilters(signStartAngle);
    }

    @Override
    public void loop() {
        double currentAngle = getCurrentAngleDeg();
        double dt = updateMotionFilters(currentAngle);
        if (dt <= 0.0) {
            return;
        }

        updateVoltageSample();

        double limitThreshold = MAX_SAFE_ANGLE - BRAKE_BUFFER_ANGLE;
        boolean hitSafetyLimit = (currentAngle >= limitThreshold && currentDirection == 1)
                || (currentAngle <= -limitThreshold && currentDirection == -1);

        if (Math.abs(currentAngle) > MAX_SAFE_ANGLE + HARD_STOP_MARGIN_ANGLE
                && !isReturningState(state)
                && state != State.CALCULATE
                && state != State.DONE
                && state != State.ERROR) {
            turret.setPower(0.0);
            errorMessage = "Past hard turret angle limit";
            switchState(State.ERROR);
        }

        switch (state) {
            case SIGN_CHECK:
                runSignCheck(currentAngle);
                break;

            case SIGN_RETURN:
                if (returnToCenter(currentAngle) || timer.milliseconds() > 3000) {
                    turret.setPower(0.0);
                    resetMotionFilters(currentAngle);
                    switchState(State.SIGN_WAIT);
                }
                break;

            case SIGN_WAIT:
                if (timer.milliseconds() >= REST_TIME_MS) {
                    switchState(State.START);
                }
                break;

            case START:
                iteration = 0;
                currentTestPower = 0.0;
                ksSamples.clear();
                kvData.clear();
                kaData.clear();
                pbData.clear();
                switchState(State.KS_RUN);
                break;

            case KS_RUN:
                runKsTest(dt, hitSafetyLimit);
                break;

            case KS_RETURN:
                if (returnToCenter(currentAngle) || timer.milliseconds() > 3000) {
                    turret.setPower(0.0);
                    resetMotionFilters(currentAngle);
                    switchState(State.KS_WAIT);
                }
                break;

            case KS_WAIT:
                if (timer.milliseconds() >= REST_TIME_MS) {
                    iteration++;
                    switchState(State.KS_RUN);
                }
                break;

            case KV_RUN:
                runKvTest(hitSafetyLimit);
                break;

            case KV_RETURN:
                if (returnToCenter(currentAngle) || timer.milliseconds() > 3000) {
                    turret.setPower(0.0);
                    resetMotionFilters(currentAngle);
                    switchState(State.KV_WAIT);
                }
                break;

            case KV_WAIT:
                if (timer.milliseconds() >= REST_TIME_MS) {
                    iteration++;
                    switchState(State.KV_RUN);
                }
                break;

            case KA_RUN:
                runKaTest(hitSafetyLimit);
                break;

            case KA_RETURN:
                if (returnToCenter(currentAngle) || timer.milliseconds() > 3000) {
                    turret.setPower(0.0);
                    resetMotionFilters(currentAngle);
                    switchState(State.KA_WAIT);
                }
                break;

            case KA_WAIT:
                if (timer.milliseconds() >= REST_TIME_MS) {
                    iteration++;
                    switchState(State.KA_RUN);
                }
                break;

            case PB_RUN:
                runPbTest(currentAngle, hitSafetyLimit);
                break;

            case PB_BRAKE:
                runPbBrake(currentAngle);
                break;

            case PB_RETURN:
                if (returnToCenter(currentAngle) || timer.milliseconds() > 3000) {
                    turret.setPower(0.0);
                    resetMotionFilters(currentAngle);
                    switchState(State.PB_WAIT);
                }
                break;

            case PB_WAIT:
                if (timer.milliseconds() >= REST_TIME_MS) {
                    iteration++;
                    switchState(State.PB_RUN);
                }
                break;

            case CALCULATE:
                calculateResults();
                switchState(State.DONE);
                break;

            case DONE:
            case ERROR:
            default:
                turret.setPower(0.0);
                break;
        }

        publishTelemetry(currentAngle);
    }

    @Override
    public void stop() {
        if (turret != null) {
            turret.setPower(0.0);
        }
    }

    private void runSignCheck(double currentAngle) {
        currentDirection = 1;
        turret.setPower(SIGN_CHECK_POWER);
        if (timer.milliseconds() < SIGN_CHECK_MS) {
            return;
        }

        turret.setPower(0.0);
        signCheckDelta = currentAngle - signStartAngle;
        if (Math.abs(signCheckDelta) < SIGN_CHECK_MIN_DELTA_DEG) {
            errorMessage = "Sign check delta too small. Increase SIGN_CHECK_POWER or check turret.";
            switchState(State.ERROR);
            return;
        }

        powerSignForPositiveAngle = signCheckDelta > 0.0 ? 1 : -1;
        switchState(State.SIGN_RETURN);
    }

    private void runKsTest(double dt, boolean hitSafetyLimit) {
        if (iteration >= 2) {
            rawKs = average(ksSamples);
            iteration = 0;
            switchState(State.KV_RUN);
            return;
        }

        currentDirection = iteration % 2 == 0 ? 1 : -1;
        currentTestPower += KS_POWER_RAMP_RATE_PER_SEC * dt;
        setPowerForAngleDirection(currentDirection, currentTestPower);

        if (Math.abs(filteredVel) > 2.0 || hitSafetyLimit || currentTestPower >= MAX_TEST_POWER) {
            ksSamples.add(Math.min(currentTestPower, MAX_TEST_POWER));
            currentTestPower = 0.0;
            turret.setPower(0.0);
            switchState(State.KS_RETURN);
        }
    }

    private void runKvTest(boolean hitSafetyLimit) {
        if (iteration >= KV_TEST_POWERS.length * 2) {
            iteration = 0;
            rawKv = calculateLeastSquaresSlope(kvData);
            switchState(State.KA_RUN);
            return;
        }

        int powerIndex = iteration / 2;
        currentDirection = iteration % 2 == 0 ? 1 : -1;
        double testPower = KV_TEST_POWERS[powerIndex];
        setPowerForAngleDirection(currentDirection, testPower);

        if (timer.milliseconds() > 500 || hitSafetyLimit) {
            if (Math.abs(filteredVel) > 3.0) {
                double netPower = Math.max(0.0, testPower - rawKs);
                kvData.add(new double[]{Math.abs(filteredVel), netPower});
            }
            turret.setPower(0.0);
            switchState(State.KV_RETURN);
        }
    }

    private void runKaTest(boolean hitSafetyLimit) {
        if (iteration >= KA_TEST_POWERS.length * 2) {
            iteration = 0;
            switchState(State.PB_RUN);
            return;
        }

        int powerIndex = iteration / 2;
        currentDirection = iteration % 2 == 0 ? 1 : -1;
        double testPower = KA_TEST_POWERS[powerIndex];
        setPowerForAngleDirection(currentDirection, testPower);

        if (timer.milliseconds() < 250 && Math.abs(filteredAccel) > 10.0) {
            double netPower = Math.max(0.0, testPower - rawKs - rawKv * Math.abs(filteredVel));
            kaData.add(new double[]{Math.abs(filteredAccel), netPower});
        }

        if (timer.milliseconds() > 350 || hitSafetyLimit) {
            turret.setPower(0.0);
            switchState(State.KA_RETURN);
        }
    }

    private void runPbTest(double currentAngle, boolean hitSafetyLimit) {
        if (iteration >= PB_TEST_POWERS.length * 2) {
            switchState(State.CALCULATE);
            return;
        }

        int powerIndex = iteration / 2;
        currentDirection = iteration % 2 == 0 ? 1 : -1;
        double testPower = PB_TEST_POWERS[powerIndex];
        setPowerForAngleDirection(currentDirection, testPower);

        if (timer.milliseconds() >= PB_SPIN_TIME_MS || hitSafetyLimit) {
            pbMeasuredBrakeVel = filteredVel;
            pbStartBrakeAngle = currentAngle;
            setPowerForAngleDirection(-currentDirection, PB_BRAKING_POWER);
            switchState(State.PB_BRAKE);
        }
    }

    private void runPbBrake(double currentAngle) {
        boolean reversedOrStopped = (Math.signum(filteredVel) != currentDirection || Math.abs(filteredVel) < 2.0)
                && timer.milliseconds() > 150;
        if (reversedOrStopped || timer.milliseconds() > 2000) {
            turret.setPower(0.0);
            double brakingDistance = Math.abs(currentAngle - pbStartBrakeAngle);
            double absVel = Math.abs(pbMeasuredBrakeVel);
            pbData.add(new double[]{absVel, brakingDistance});
            switchState(State.PB_RETURN);
        }
    }

    private double getCurrentAngleDeg() {
        return turret.getCurrentPosition() / TURRET_FULL_RANGE_ENCODER * TURRET_FULL_RANGE_DEGREE;
    }

    private double updateMotionFilters(double currentAngle) {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        if (dt <= 0.00001) {
            return 0.0;
        }
        lastTime = currentTime;

        if (!isInitialized) {
            resetMotionFilters(currentAngle);
            return 0.0;
        }

        double rawVel = (currentAngle - lastAngle) / dt;
        filteredVel = VEL_FILTER_ALPHA * rawVel + (1.0 - VEL_FILTER_ALPHA) * filteredVel;

        double rawAccel = (filteredVel - lastFilteredVel) / dt;
        filteredAccel = ACCEL_FILTER_ALPHA * rawAccel + (1.0 - ACCEL_FILTER_ALPHA) * filteredAccel;

        lastAngle = currentAngle;
        lastFilteredVel = filteredVel;
        return dt;
    }

    private void resetMotionFilters(double currentAngle) {
        lastAngle = currentAngle;
        filteredVel = 0.0;
        lastFilteredVel = 0.0;
        filteredAccel = 0.0;
        lastTime = System.nanoTime();
        isInitialized = true;
    }

    private void updateVoltageSample() {
        if (voltageTimer.milliseconds() <= 250 || state == State.DONE || state == State.ERROR || state == State.CALCULATE) {
            return;
        }
        currentVoltage = getBatteryVoltage();
        sumVoltage += currentVoltage;
        voltageSamples++;
        voltageTimer.reset();
    }

    private double getBatteryVoltage() {
        double maxVoltage = 0.0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > maxVoltage) {
                maxVoltage = voltage;
            }
        }
        return maxVoltage > 0.0 ? maxVoltage : 12.0;
    }

    private void setPowerForAngleDirection(int angleDirection, double magnitude) {
        double power = powerSignForPositiveAngle * angleDirection * Math.abs(magnitude);
        turret.setPower(Range.clip(power, -MAX_TEST_POWER, MAX_TEST_POWER));
    }

    private boolean returnToCenter(double currentAngle) {
        if (Math.abs(currentAngle) > RETURN_TOLERANCE) {
            setPowerForAngleDirection((int) -Math.signum(currentAngle), RETURN_POWER);
            return false;
        }
        turret.setPower(0.0);
        return true;
    }

    private boolean isReturningState(State currentState) {
        return currentState == State.SIGN_RETURN
                || currentState == State.KS_RETURN
                || currentState == State.KV_RETURN
                || currentState == State.KA_RETURN
                || currentState == State.PB_RETURN
                || currentState == State.PB_BRAKE;
    }

    private void calculateResults() {
        avgTestVoltage = voltageSamples > 0 ? sumVoltage / voltageSamples : getBatteryVoltage();
        rawKa = calculateLeastSquaresSlope(kaData);
        calculatePbCoefficients();

        double voltageScaleRatio = avgTestVoltage / Math.max(1.0, REFERENCE_VOLTAGE);
        compKs = rawKs * voltageScaleRatio;
        compKv = rawKv * voltageScaleRatio;
        compKa = Math.max(0.0, rawKa * voltageScaleRatio);
        compKLin = Math.max(0.0, rawKLin * voltageScaleRatio);
        compKQuad = Math.max(0.0, rawKQuad * voltageScaleRatio);

        AutoAimSubsystem.TURRET_POWER_SIGN = powerSignForPositiveAngle;
        AutoAimSubsystem.TURRET_kS = compKs;
        AutoAimSubsystem.TURRET_kV = compKv;
        AutoAimSubsystem.TURRET_kA = compKa;
        AutoAimSubsystem.TURRET_kLinearBraking = compKLin;
        AutoAimSubsystem.TURRET_kQuadraticFriction = compKQuad;
    }

    private double average(List<Double> data) {
        if (data.isEmpty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (double value : data) {
            sum += value;
        }
        return sum / data.size();
    }

    private double calculateLeastSquaresSlope(List<double[]> data) {
        double sumXY = 0.0;
        double sumXX = 0.0;
        for (double[] point : data) {
            sumXY += point[0] * point[1];
            sumXX += point[0] * point[0];
        }
        return sumXX <= 0.0 ? 0.0 : sumXY / sumXX;
    }

    private void calculatePbCoefficients() {
        double sumX = 0.0;
        double sumY = 0.0;
        double sumXY = 0.0;
        double sumXX = 0.0;
        int n = 0;

        for (double[] point : pbData) {
            double velocity = point[0];
            double distance = point[1];
            if (velocity < 5.0) {
                continue;
            }

            double x = velocity;
            double y = distance / velocity;
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumXX += x * x;
            n++;
        }

        if (n > 1) {
            double denominator = n * sumXX - sumX * sumX;
            if (Math.abs(denominator) > 1e-9) {
                rawKQuad = Math.max(0.0, (n * sumXY - sumX * sumY) / denominator);
                rawKLin = Math.max(0.0, (sumY * sumXX - sumX * sumXY) / denominator);
            }
        }
    }

    private void switchState(State newState) {
        state = newState;
        timer.reset();
    }

    private void publishTelemetry(double currentAngle) {
        telemetry.addData("State", state);
        telemetry.addData("Angle deg", "%.2f", currentAngle);
        telemetry.addData("Velocity deg/s", "%.2f", filteredVel);
        telemetry.addData("Accel deg/s/s", "%.2f", filteredAccel);
        telemetry.addData("+Power -> +Angle sign", powerSignForPositiveAngle);
        telemetry.addData("Sign check delta", "%.3f", signCheckDelta);
        telemetry.addData("Voltage", "%.2f V", currentVoltage);
        telemetry.addData("Progress", "%d", iteration);

        if (state == State.ERROR) {
            telemetry.addData("ERROR", errorMessage);
            telemetry.addLine("Fix the issue, stop, and restart this OpMode.");
        } else if (state == State.DONE) {
            telemetry.addLine("=== TUNING COMPLETE ===");
            telemetry.addData("Data Points", "kS:%d kV:%d kA:%d PB:%d", ksSamples.size(), kvData.size(), kaData.size(), pbData.size());
            telemetry.addData("Avg Test Voltage", "%.2f V", avgTestVoltage);
            telemetry.addData("REFERENCE_VOLTAGE", "%.2f V", REFERENCE_VOLTAGE);
            telemetry.addData("AutoAimSubsystem.TURRET_POWER_SIGN", "%d", powerSignForPositiveAngle);
            telemetry.addData("AutoAimSubsystem.TURRET_kS", "%.6f", compKs);
            telemetry.addData("AutoAimSubsystem.TURRET_kV", "%.6f", compKv);
            telemetry.addData("AutoAimSubsystem.TURRET_kA", "%.6f", compKa);
            telemetry.addData("AutoAimSubsystem.TURRET_kLinearBraking", "%.6f", compKLin);
            telemetry.addData("AutoAimSubsystem.TURRET_kQuadraticFriction", "%.6f", compKQuad);
        }

        telemetry.update();
    }
}
