// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Swerve Modules
    public static final double SWERVE_CENTER_DISTANCE = 0.4; // Meters
    public static final double SWERVE_MAX_ANGULAR_VELOCITY = 360; // Degrees per second
    public static final double SWERVE_MAX_ANGULAR_ACCEL = 180; // Degrees per second squared
    public static final double SWERVE_DEGREES_PER_PULSE = 1;
    public static final double SWERVE_ROTATION_P = 0;
    public static final double SWERVE_ROTATION_I = 0;
    public static final double SWERVE_ROTATION_D = 0;
    public static final double SWERVE_ROTATION_KV = 12.5 / SWERVE_MAX_ANGULAR_VELOCITY;
    public static final double SWERVE_ROTATION_KA = 0;
    public static final double SWERVE_MAX_VELOCITY = 12;
    public static final double SWERVE_METERS_PER_PULSE = 0.001;
    public static final double SWERVE_DRIVE_P = 0;
    public static final double SWERVE_DRIVE_I = 0;
    public static final double SWERVE_DRIVE_D = 0;

    public static final int SWERVE_DRIVE_CHANNEL_FL = 0;
    public static final int SWERVE_ROTATION_CHANNEL_FL = 0;
    public static final int SWERVE_ENCODER0_CHANNEL_FL = 0;
    public static final int SWERVE_ENCODER1_CHANNEL_FL = 1;
    public static final int SWERVE_DRIVE_CHANNEL_FR = 1;
    public static final int SWERVE_ROTATION_CHANNEL_FR = 1;
    public static final int SWERVE_ENCODER0_CHANNEL_FR = 2;
    public static final int SWERVE_ENCODER1_CHANNEL_FR = 3;
    public static final int SWERVE_DRIVE_CHANNEL_BL = 2;
    public static final int SWERVE_ROTATION_CHANNEL_BL = 2;
    public static final int SWERVE_ENCODER0_CHANNEL_BL = 4;
    public static final int SWERVE_ENCODER1_CHANNEL_BL = 5;
    public static final int SWERVE_DRIVE_CHANNEL_BR = 3;
    public static final int SWERVE_ROTATION_CHANNEL_BR = 3;
    public static final int SWERVE_ENCODER0_CHANNEL_BR = 6;
    public static final int SWERVE_ENCODER1_CHANNEL_BR = 7;

    // Drive Subsystem
    public static final int DRIVE_PIGEON_CHANNEL = 0;

    //Pathfinding
    public static final double DRIVE_X_CONTROLLER_P = 0;
    public static final double DRIVE_X_CONTROLLER_I = 0;
    public static final double DRIVE_X_CONTROLLER_D = 0;
    public static final double DRIVE_Y_CONTROLLER_P = 0;
    public static final double DRIVE_Y_CONTROLLER_I = 0;
    public static final double DRIVE_Y_CONTROLLER_D = 0;
    public static final double DRIVE_ROTATION_CONTROLLER_P = 0;
    public static final double DRIVE_ROTATION_CONTROLLER_I = 0;
    public static final double DRIVE_ROTATION_CONTROLLER_D = 0;
    public static final double DRIVE_MAX_ROTATION_VELOCITY = 0; //Degrees Per Second
    public static final double DRIVE_MAX_ROTATION_ACCLERATION = 0; //Degrees Per Second Squared

    public static final int HOOD_MOTOR_CHANNEL = 0;
    public static final double HOOD_CONTROLLER_P = 0;
    public static final double HOOD_CONTROLLER_I = 0;
    public static final double HOOD_CONTROLLER_D = 0;
    public static final double HOOD_DEGREES_PER_ROTATION = 0;
    // Carousel Constants
    public static final int CAROUSEL_MOTOR1 = 2; // PWM: VICTOR SP
    public static final int CAROUSEL_MOTOR2 = 7; // PWM: VICTOR SP
    public static final int CAROUSEL_MOTOR3 = 6; // PWM: VICTOR SP
    public static final int CAROUSEL_MOTOR4 = 5; // PWM: VICTOR SP
    public static final int CAROUSEL_MOTOR5 = 4; // PWM: VICTOR SP
}
