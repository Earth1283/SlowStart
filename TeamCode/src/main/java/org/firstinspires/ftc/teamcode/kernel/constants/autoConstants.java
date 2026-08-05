package org.firstinspires.ftc.teamcode.kernel.constants;

import com.pedropathing.geometry.Pose;

public class autoConstants {
    public static long TOTAL_SHOOT_TIME = 0;

    public static long AUTO_CLOSE_WAIT_FOR_SHOOT = 0;
    public static double CLOSE_FIRE_DISTANCE = 68.5;
    public static double CLOSE_FIRE_DISTANCE_LAST = 60;
    public static double CLOSE_HOLD_DISTANCE = 50.0;
    public static double RED_CLOSE_FIRE_TURRET = 47;
    public static double RED_CLOSE_FIRE_TURRET_LAST = 0;

    public static double RED_INTAKE_START_X = 99;
    public static double RED_INTAKE_END_X = 120;
    public static double RED_INTAKE_FIRST_ROW_Y = 84;
    public static double RED_INTAKE_SECOND_ROW_Y = 59.5;
    public static double RED_INTAKE_THIRD_ROW_Y = 36;

    public static Pose RED_CLOSE_START = new Pose(124.283, 119.728, Math.toRadians(37.54));
    public static Pose RED_CLOSE_SHOOT = new Pose(90, 90, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT_CONTROL = new Pose(88, 58);
    public static Pose RED_CLOSE_SHOOT_PRELOAD = new Pose(RED_CLOSE_SHOOT.getX(), RED_CLOSE_SHOOT.getY(), RED_CLOSE_SHOOT.getHeading());
    public static Pose RED_CLOSE_INTAKE_FIRST_CONTROL = new Pose(94, 78);
    public static Pose RED_CLOSE_INTAKE_FIRST_END = new Pose(RED_INTAKE_END_X, RED_INTAKE_FIRST_ROW_Y, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT_FIRST_ROW = new Pose(84.5, 106, Math.toRadians(31));
    public static Pose RED_CLOSE_INTAKE_SECOND_CONTROL = new Pose(86, 52);
    public static Pose RED_CLOSE_INTAKE_SECOND_END = new Pose(RED_INTAKE_END_X, RED_INTAKE_SECOND_ROW_Y, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT_SECOND_ROW = new Pose(RED_CLOSE_SHOOT.getX(), RED_CLOSE_SHOOT.getY(), RED_CLOSE_SHOOT.getHeading());
    public static Pose RED_CLOSE_INTAKE_THIRD_START = new Pose(RED_INTAKE_START_X, RED_INTAKE_THIRD_ROW_Y, Math.toRadians(0));
    public static Pose RED_CLOSE_INTAKE_THIRD_END = new Pose(RED_INTAKE_END_X, RED_INTAKE_THIRD_ROW_Y, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT_THIRD_ROW = new Pose(RED_CLOSE_SHOOT.getX(), RED_CLOSE_SHOOT.getY(), RED_CLOSE_SHOOT.getHeading());
    public static Pose RED_CLOSE_INTAKE_GATE = new Pose(130, 59.5, Math.toRadians(29.5));
    public static Pose RED_CLOSE_SHOOT_INTAKE_GATE = new Pose(RED_CLOSE_SHOOT.getX(), RED_CLOSE_SHOOT.getY(), RED_CLOSE_SHOOT.getHeading());

    public static Pose RED_CLOSE_OPEN_GATE_2 = new Pose(127, 66, Math.toRadians(0));
    public static Pose RED_CLOSE_OPEN_GATE_2_CONTROL_POINT = new Pose(120, 66);

    public static double BLUE_CLOSE_FIRE_TURRET = -47;
    public static double BLUE_CLOSE_FIRE_TURRET_LAST = 0;

    public static double BLUE_INTAKE_START_X = 45;
    public static double BLUE_INTAKE_END_X = 24;
    public static double BLUE_INTAKE_FIRST_ROW_Y = 84;
    public static double BLUE_INTAKE_SECOND_ROW_Y = 59.5;
    public static double BLUE_INTAKE_THIRD_ROW_Y = 36;

    public static Pose BLUE_CLOSE_START = new Pose(19.717, 119.728, Math.toRadians(142.46));
    public static Pose BLUE_CLOSE_SHOOT = new Pose(54, 90, Math.toRadians(180));
    public static Pose BLUE_CLOSE_SHOOT_CONTROL = new Pose(56, 58);
    public static Pose BLUE_CLOSE_SHOOT_PRELOAD = new Pose(BLUE_CLOSE_SHOOT.getX(), BLUE_CLOSE_SHOOT.getY(), BLUE_CLOSE_SHOOT.getHeading());
    public static Pose BLUE_CLOSE_INTAKE_FIRST_CONTROL = new Pose(50, 78);
    public static Pose BLUE_CLOSE_INTAKE_FIRST_END = new Pose(BLUE_INTAKE_END_X, BLUE_INTAKE_FIRST_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_CLOSE_SHOOT_FIRST_ROW = new Pose(59.5, 106, Math.toRadians(149));
    public static Pose BLUE_CLOSE_INTAKE_SECOND_CONTROL = new Pose(58, 52);
    public static Pose BLUE_CLOSE_INTAKE_SECOND_END = new Pose(BLUE_INTAKE_END_X, BLUE_INTAKE_SECOND_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_CLOSE_SHOOT_SECOND_ROW = new Pose(BLUE_CLOSE_SHOOT.getX(), BLUE_CLOSE_SHOOT.getY(), BLUE_CLOSE_SHOOT.getHeading());
    public static Pose BLUE_CLOSE_INTAKE_THIRD_START = new Pose(BLUE_INTAKE_START_X, BLUE_INTAKE_THIRD_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_CLOSE_INTAKE_THIRD_END = new Pose(BLUE_INTAKE_END_X, BLUE_INTAKE_THIRD_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_CLOSE_SHOOT_THIRD_ROW = new Pose(BLUE_CLOSE_SHOOT.getX(), BLUE_CLOSE_SHOOT.getY(), BLUE_CLOSE_SHOOT.getHeading());
    public static Pose BLUE_CLOSE_INTAKE_GATE = new Pose(14, 59.5, Math.toRadians(150.5));
    public static Pose BLUE_CLOSE_SHOOT_INTAKE_GATE = new Pose(BLUE_CLOSE_SHOOT.getX(), BLUE_CLOSE_SHOOT.getY(), BLUE_CLOSE_SHOOT.getHeading());

    public static Pose BLUE_CLOSE_OPEN_GATE_2 = new Pose(12.5, 66, Math.toRadians(180));
    public static Pose BLUE_CLOSE_OPEN_GATE_2_CONTROL_POINT = new Pose(24, 66);

    public static long AUTO_FAR_WAIT_FOR_SHOOT = 400;
    public static double FAR_FIRE_DISTANCE = 126.5;
    public static double FAR_FIRE_DISTANCE_PRELOAD = 134;
    public static double FAR_HOLD_DISTANCE = 126.5;
    public static double RED_FAR_TURRET = 70;
    public static double RED_FAR_TURRET_PRELOAD = 70.5;

    public static Pose RED_FAR_START = new Pose(86.834, 7.362, Math.toRadians(0));
    public static Pose RED_FAR_SHOOT = new Pose(85.0, 17, Math.toRadians(0));
    public static Pose RED_FAR_INTAKE_HP_1_START = new Pose(120, 8, Math.toRadians(0));
    public static Pose RED_FAR_INTAKE_HP_1_MID = new Pose(132.5, 9, Math.toRadians(-20));
    public static Pose RED_FAR_INTAKE_HP_1_END = new Pose(131.5, 15, Math.toRadians(20));
    public static Pose RED_FAR_SHOOT_HP = new Pose(RED_FAR_SHOOT.getX(), RED_FAR_SHOOT.getY(), RED_FAR_SHOOT.getHeading());
    public static Pose RED_FAR_SHOOT_TUNNEL = new Pose(RED_FAR_SHOOT.getX(), RED_FAR_SHOOT.getY(), RED_FAR_SHOOT.getHeading());
    public static Pose RED_FAR_PARK = new Pose(96, 27, Math.toRadians(0));
    public static Pose RED_FAR_INTAKE_THIRD_CONTROL = new Pose(86, 38);
    public static Pose RED_FAR_INTAKE_THIRD_END = new Pose(118.0, 35.5, Math.toRadians(0.0));
    public static Pose RED_FAR_INTAKE_TUNNEL_START = new Pose(90.0, 26.5, Math.toRadians(0.0));
    public static Pose RED_FAR_INTAKE_TUNNEL_END = new Pose(132.0, 26.5, Math.toRadians(15.0));

    public static double BLUE_FAR_TURRET = -70;
    public static double BLUE_FAR_TURRET_PRELOAD = -70.5;
    public static Pose BLUE_FAR_START = new Pose(57.166, 7.362, Math.toRadians(180.0));
    public static Pose BLUE_FAR_SHOOT = new Pose(59.0, 17.0, Math.toRadians(180.0));
    public static Pose BLUE_FAR_INTAKE_HP_1_START = new Pose(24.0, 8.0, Math.toRadians(180.0));
    public static Pose BLUE_FAR_INTAKE_HP_1_MID = new Pose(11.5, 9.0, Math.toRadians(-160.0));
    public static Pose BLUE_FAR_INTAKE_HP_1_END = new Pose(12.5, 15.0, Math.toRadians(160.0));
    public static Pose BLUE_FAR_SHOOT_HP = new Pose(BLUE_FAR_SHOOT.getX(), BLUE_FAR_SHOOT.getY(), BLUE_FAR_SHOOT.getHeading());
    public static Pose BLUE_FAR_SHOOT_TUNNEL = new Pose(BLUE_FAR_SHOOT.getX(), BLUE_FAR_SHOOT.getY(), BLUE_FAR_SHOOT.getHeading());
    public static Pose BLUE_FAR_PARK = new Pose(48.0, 27.0, Math.toRadians(180.0));
    public static Pose BLUE_FAR_INTAKE_THIRD_CONTROL = new Pose(58.0, 38.0);
    public static Pose BLUE_FAR_INTAKE_THIRD_END = new Pose(26.0, 35.5, Math.toRadians(180.0));
    public static Pose BLUE_FAR_INTAKE_TUNNEL_START = new Pose(54.0, 26.5, Math.toRadians(180.0));
    public static Pose BLUE_FAR_INTAKE_TUNNEL_END = new Pose(12.0, 26.5, Math.toRadians(165.0));
}
