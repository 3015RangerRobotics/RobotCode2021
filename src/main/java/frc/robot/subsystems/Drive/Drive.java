// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {

  Translation2d frontLeftLocation = new Translation2d(Constants.SWERVE_CENTER_DISTANCE, Constants.SWERVE_CENTER_DISTANCE);
  Translation2d frontRightLocation = new Translation2d(Constants.SWERVE_CENTER_DISTANCE, -Constants.SWERVE_CENTER_DISTANCE);
  Translation2d backLeftLocation = new Translation2d(-Constants.SWERVE_CENTER_DISTANCE, Constants.SWERVE_CENTER_DISTANCE);
  Translation2d backRightLocation = new Translation2d(-Constants.SWERVE_CENTER_DISTANCE, -Constants.SWERVE_CENTER_DISTANCE);

  SwerveModule frontLeftSwerveModule;
  SwerveModule frontRightSwerveModule;
  SwerveModule backLeftSwerveModule;
  SwerveModule backRightSwerveModule;

  PigeonIMU imu;

  SwerveDriveKinematics kinematics;
  SwerveDriveOdometry odometry;

  /** Creates a new Drive. */
  public Drive() {
    frontLeftSwerveModule = new SwerveModule(Constants.SWERVE_DRIVE_CHANNEL_FL, Constants.SWERVE_ROTATION_CHANNEL_FL, Constants.SWERVE_ENCODER0_CHANNEL_FL, Constants.SWERVE_ENCODER1_CHANNEL_FL);
    frontRightSwerveModule = new SwerveModule(Constants.SWERVE_DRIVE_CHANNEL_FR, Constants.SWERVE_ROTATION_CHANNEL_FR, Constants.SWERVE_ENCODER0_CHANNEL_FR, Constants.SWERVE_ENCODER1_CHANNEL_FR);
    backLeftSwerveModule = new SwerveModule(Constants.SWERVE_DRIVE_CHANNEL_BL, Constants.SWERVE_ROTATION_CHANNEL_BL, Constants.SWERVE_ENCODER0_CHANNEL_BL, Constants.SWERVE_ENCODER1_CHANNEL_BL);
    backRightSwerveModule = new SwerveModule(Constants.SWERVE_DRIVE_CHANNEL_BR, Constants.SWERVE_ROTATION_CHANNEL_BR, Constants.SWERVE_ENCODER0_CHANNEL_BR, Constants.SWERVE_ENCODER1_CHANNEL_BR);
    resetEncoders();

    imu = new PigeonIMU(Constants.DRIVE_PIGEON_CHANNEL);
    imu.configFactoryDefault();
    resetIMU();

    kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    odometry = new SwerveDriveOdometry(kinematics, getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEncoders() {
    frontLeftSwerveModule.resetEncoders();
    frontRightSwerveModule.resetEncoders();
    backLeftSwerveModule.resetEncoders();
    backRightSwerveModule.resetEncoders();
  }

  public void resetIMU() {
    imu.setYaw(0);
  }

  public double getAngleDegrees() {
    return imu.getFusedHeading();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(imu.getFusedHeading());
  }

  public void updateOdometry() {
    odometry.update(getRotation2d(), frontLeftSwerveModule.getSwerveModuleState(), frontRightSwerveModule.getSwerveModuleState(), backLeftSwerveModule.getSwerveModuleState(), backRightSwerveModule.getSwerveModuleState());
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, getRotation2d());
  }

  public void setModuleDrivePct(double pct) {
    frontLeftSwerveModule.setDriveMotor(ControlMode.PercentOutput, pct);
    frontRightSwerveModule.setDriveMotor(ControlMode.PercentOutput, pct);
    backLeftSwerveModule.setDriveMotor(ControlMode.PercentOutput, pct);
    backRightSwerveModule.setDriveMotor(ControlMode.PercentOutput, pct);
  }

  public void setModuleRotationVoltage(double voltage) {
    frontLeftSwerveModule.setRotationMotorVoltage(voltage);
    frontRightSwerveModule.setRotationMotorVoltage(voltage);
    backLeftSwerveModule.setRotationMotorVoltage(voltage);
    backRightSwerveModule.setRotationMotorVoltage(voltage);
  }

  public void setModuleRotationStationary(double degrees) {
    SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(degrees));
    frontLeftSwerveModule.setSwerveModuleState(state);
    frontRightSwerveModule.setSwerveModuleState(state);
    backLeftSwerveModule.setSwerveModuleState(state);
    backRightSwerveModule.setSwerveModuleState(state);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    frontLeftSwerveModule.setSwerveModuleState(moduleStates[0]);
    frontRightSwerveModule.setSwerveModuleState(moduleStates[1]);
    backLeftSwerveModule.setSwerveModuleState(moduleStates[2]);
    backRightSwerveModule.setSwerveModuleState(moduleStates[3]);
  }

  public void drive(double xVelMeters, double yVelMeters, double degreesPerSecond, boolean isFieldRelative) {
    if(isFieldRelative) {
      drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVelMeters, yVelMeters, Math.toRadians(degreesPerSecond), getRotation2d()));
    } else {
      drive(new ChassisSpeeds(xVelMeters, yVelMeters, Math.toRadians(degreesPerSecond)));
    }
  }
}
