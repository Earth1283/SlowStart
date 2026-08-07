package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.TOTAL_SHOOT_TIME;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

/**
 * Team 32008's own Robot aggregate, copied from FTC-32008 V2 subsystems/Robot.java.
 *
 * ONLY change from their file: the constants import points at kernel.constants.
 * Every init order, subsystem and timing below is theirs, unedited.
 *
 * Note autoInit() does NOT init the drivetrain -- in auto, Pedro's follower owns
 * the drive motors and the Pinpoint, so a second owner here would fight it.
 */
public class Robot {
    ScheduledExecutorService exec = Executors.newScheduledThreadPool(50);
    public Drivetrain drivetrain = new Drivetrain();
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();
    public Indicator indicator = new Indicator();

    public void init(HardwareMap hardwareMap) {
        exec = Executors.newScheduledThreadPool(5);
        drivetrain.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        indicator.init(hardwareMap);
    }

    public void init(HardwareMap hardwareMap, boolean aass) {
        exec = Executors.newScheduledThreadPool(5);
        drivetrain.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap, aass);
        indicator.init(hardwareMap);
    }

    public void autoInit(HardwareMap hardwareMap) {
        exec = Executors.newScheduledThreadPool(5);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        shooter.reset();
        indicator.init(hardwareMap);
    }

    public void shootAll(long shoot, long gap) {
        intake.intakeIn();
        TOTAL_SHOOT_TIME = shoot * 3 + gap * 2 + 200;
        if (gap > 100) exec.schedule(() -> intake.intakeInSlow(), shoot, java.util.concurrent.TimeUnit.MILLISECONDS);
        exec.schedule(() -> intake.intakeIn(), shoot + gap, java.util.concurrent.TimeUnit.MILLISECONDS);
        if (gap > 100) exec.schedule(() -> intake.intakeInSlow(), shoot * 2 + gap, java.util.concurrent.TimeUnit.MILLISECONDS);
        exec.schedule(() -> intake.intakeIn(), shoot * 2 + gap * 2, java.util.concurrent.TimeUnit.MILLISECONDS);
        if (gap > 100) exec.schedule(() -> intake.intakeStop(), shoot * 3 + gap * 2 + 200, java.util.concurrent.TimeUnit.MILLISECONDS);
    }

    public void prepareShootAuto(double distance, double turret) {
        intake.gateOpen();
        shooter.setShooterByDis(distance);
        shooter.turretToDegree(turret);
    }

    public void stopShootAuto() {
        intake.gateClose();
        shooter.shooterStop();
    }
}
