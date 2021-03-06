// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import lib.LookupTable;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Swerve Module Specific
    public static final double SWERVE_CENTER_DISTANCE = 0.2942971; // Meters
    public static final double SWERVE_DEGREES_PER_PULSE = 360.0 / 4096.0;
    public static final double SWERVE_ROTATION_P = 16; // 16
    public static final double SWERVE_ROTATION_I = .04; // 0.02
    public static final double SWERVE_ROTATION_I_ZONE = 10 / SWERVE_DEGREES_PER_PULSE;
    public static final double SWERVE_ROTATION_D = 1600; // 500
    public static final double SWERVE_MAX_VELOCITY = 4.5;
    public static final double SWERVE_METERS_PER_PULSE = 0.00002226;
    public static final double SWERVE_DRIVE_P = -1000; // For some reason this doesn't work with positive constants
    public static final double SWERVE_DRIVE_I = 0;
    public static final double SWERVE_DRIVE_D = -25;

    public static final int SWERVE_DRIVE_CHANNEL_FL = 1;
    public static final int SWERVE_ROTATION_CHANNEL_FL = 5;
    public static final int SWERVE_DRIVE_CHANNEL_FR = 2;
    public static final int SWERVE_ROTATION_CHANNEL_FR = 6;
    public static final int SWERVE_DRIVE_CHANNEL_BL = 3;
    public static final int SWERVE_ROTATION_CHANNEL_BL = 7;
    public static final int SWERVE_DRIVE_CHANNEL_BR = 4;
    public static final int SWERVE_ROTATION_CHANNEL_BR = 8;

    // Drive Subsystem
    public static final int DRIVE_PIGEON_CHANNEL = 1;
    public static final double DRIVE_MAX_ANGULAR_VELOCITY = 600;
    public static final double DRIVE_MAX_ANGULAR_ACCEL = 6000;

    // Path Following
    public static final double DRIVE_POS_ERROR_CONTROLLER_P = 12; // 10
    public static final double DRIVE_POS_ERROR_CONTROLLER_I = 0;
    public static final double DRIVE_POS_ERROR_CONTROLLER_D = 0.05;
    public static final double DRIVE_HEADING_ERROR_CONTROLLER_P = 0; // 1.05
    public static final double DRIVE_HEADING_ERROR_CONTROLLER_I = 0;
    public static final double DRIVE_HEADING_ERROR_CONTROLLER_D = 0; // 0.02
    public static final double DRIVE_ROTATION_CONTROLLER_P = 10;// 9
    public static final double DRIVE_ROTATION_CONTROLLER_I = 0;
    public static final double DRIVE_ROTATION_CONTROLLER_D = 0;
    public static final double DRIVE_TARGETING_CONTROLLER_P = 13;// 9
    public static final double DRIVE_TARGETING_CONTROLLER_I = 0;
    public static final double DRIVE_TARGETING_CONTROLLER_D = 0.5;
    public static final double DRIVE_ROTATION_MIN_VELOCITY = 25;
    public static final double DRIVE_TARGETING_I_ZONE = 2;

    // Hood Constants
    public static final int HOOD_MOTOR_CHANNEL = 1;
    public static final double HOOD_CONTROLLER_P = .5;
    public static final double HOOD_CONTROLLER_I = 0;
    public static final double HOOD_CONTROLLER_D = 0;
    public static final double HOOD_DEGREES_PER_ROTATION = 0.2847;
    public static final LookupTable HOOD_AUTO_POSITION_TABLE = new LookupTable();

    static {
        // Mapping from distance to target to hood angle necessary
        // to make a shot.
        HOOD_AUTO_POSITION_TABLE.put(6.7, 12);
        HOOD_AUTO_POSITION_TABLE.put(8.5, 17);
        HOOD_AUTO_POSITION_TABLE.put(10.3, 20.6);
        HOOD_AUTO_POSITION_TABLE.put(11.8, 23);
        HOOD_AUTO_POSITION_TABLE.put(12.9, 23.7);
        HOOD_AUTO_POSITION_TABLE.put(14.4, 25);
        HOOD_AUTO_POSITION_TABLE.put(15.7, 27);
        HOOD_AUTO_POSITION_TABLE.put(17.6, 28);
    }

    // Carousel Constants
    public static final int CAROUSEL_MOTOR1 = 0; // PWM: VICTOR SPX
    public static final int CAROUSEL_MOTOR2 = 1; // PWM: VICTOR SPX
    public static final int CAROUSEL_MOTOR3 = 2; // PWM: VICTOR SPX
    public static final int CAROUSEL_INTAKE_MOTOR = 3; // PWM: VICTOR SP
    public static final int CAROUSEL_BALL_SENSOR1 = 0; // DIO: BEAM BREAK
    public static final int CAROUSEL_BALL_SENSOR2 = 1; // DIO: BEAM BREAK
    public static final int CAROUSEL_BALL_SENSOR3 = 2; // DIO: BEAM BREAK
    public static final int CAROUSEL_ENCODER1 = 3;
    public static final int CAROUSEL_ENCODER2 = 4;
    public static final int CAROUSEL_ENCODER3 = 5;
    public static final double CAROUSEL_IN_SPEED_ACTIVE = 0.4;
    public static final double CAROUSEL_IN_SPEED_PASSIVE = 1;
    public static final double CAROUSEL_PURGE_SPEED1 = -0.8;
    public static final double CAROUSEL_PURGE_SPEED2 = -0.9;
    public static final double CAROUSEL_PURGE_SPEED3 = -1;
    public static final double CAROUSEL_SHOOT_PERCENTAGE = 1.0;
    public static final double CAROUSEL_SHOOT_VELOCITY = 50000;
    public static final double CAROUSEL_SHOOT_P = 0.00002;
    public static final double CAROUSEL_SHOOT_I = 0;
    public static final double CAROUSEL_SHOOT_D = 0;
    public static final double CAROUSEL_SHOOT_F = 1.0 / 70000;

    // Limelight Constants
    public static final double LL_TARGET_HEIGHT = 7.58;
    public static final double LL_MOUNT_HEIGHT = 1.625;
    public static final double LL_MOUNT_ANGLE = 30;

    // Shooter Constants
    public static final int SHOOTER_MOTOR = 10; // CAN: TALON FX
    public static final int SHOOTER_MOTOR_2 = 11; // CAN: TALON FX
    public static final double SHOOTER_P = 0.4;// 0.4
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 10;// 10
    public static final double SHOOTER_F = 0.0455;// 0.0468
    public static final double SHOOTER_PULSES_PER_ROTATION = 2048 * (2.0 / 3.0);
    public static final double SHOOTER_SHOOT_P = SHOOTER_P;// 25
    public static final double SHOOTER_SHOOT_I = SHOOTER_I;
    public static final double SHOOTER_SHOOT_D = SHOOTER_D;// 60
    public static final double SHOOTER_SHOOT_F = SHOOTER_F;
    public static final double SHOOTER_LAUNCH_ANGLE = 53.28;
    public static final double SHOOTER_TOLERANCE = 25;
    public static final LookupTable SHOOTER_AUTO_SPEED_TABLE = new LookupTable();

    static {
        SHOOTER_AUTO_SPEED_TABLE.put(0, 0);
    }

    // Turret Constants
    public static final int TURRET_MOTOR = 6; // CAN: TALON SRX
    public static final int TURRET_LEFT_LIMIT = 1; // DIO: LIMIT SWITCH
    public static final int TURRET_RIGHT_LIMIT = 0; // DIO: LIMIT SWITCH
    public static final double TURRET_P = 0.8;
    public static final double TURRET_I = 0;
    public static final double TURRET_D = 20;
    public static final double TURRET_F = 0;
    public static final double TURRET_DEGREES_PER_PULSE = 1 / (5600.0 / 90.0);
    public static final double TURRET_MAX_SPEED = 0.7;
    public static final double TURRET_MIN_SPEED = 0.1;
    public static final double TURRET_ALLOWABLE_ERROR = Math.round((1 / Constants.TURRET_DEGREES_PER_PULSE) * 1);
    public static final int TURRET_HOMING_POSITION_LEFT = (int) Math.round(-110 / TURRET_DEGREES_PER_PULSE);
    public static final int TURRET_HOMING_POSITION_RIGHT = (int) Math.round(52 / TURRET_DEGREES_PER_PULSE);
}
