// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import lib.swerve.ObstacleAvoidance;

public class DriveWithGamepad extends CommandBase {
    ProfiledPIDController rotationController;
    ObstacleAvoidance roomba;
    double currentAngle;

    /**
     * Creates a new DriveWithGamepad.
     */
    public DriveWithGamepad() {
        addRequirements(RobotContainer.drive);
        rotationController = new ProfiledPIDController(Constants.DRIVE_ROTATION_CONTROLLER_P, Constants.DRIVE_ROTATION_CONTROLLER_I, Constants.DRIVE_ROTATION_CONTROLLER_D,
                new TrapezoidProfile.Constraints(Constants.DRIVE_MAX_ANGULAR_VELOCITY, Constants.DRIVE_MAX_ANGULAR_ACCEL));
        rotationController.enableContinuousInput(-180, 180);
        roomba = new ObstacleAvoidance(0.45, 4,
                new ObstacleAvoidance.RestrictedArea(new Translation2d(0.5, -5), new Translation2d(2, 5))
        );
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentAngle = RobotContainer.drive.getAngleDegrees();
        rotationController.reset(currentAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double rightStickX = RobotContainer.getDriverRightStickX();
        double leftStickY = RobotContainer.getDriverLeftStickY() * (RobotContainer.getDriverRightTrigger() > 0.5 ? 1 : 1);
        double leftStickX = RobotContainer.getDriverLeftStickX() * (RobotContainer.getDriverRightTrigger() > 0.5 ? 1 : 1);

        double rotationOutput = rightStickX;
        if(Math.abs(rotationOutput) == 0){
            double rot = (RobotContainer.getDriverLeftTrigger() > 0.5) ? currentAngle - 25 : currentAngle;
            rotationOutput = rotationController.calculate(RobotContainer.drive.getAngleDegrees(), rot);
            if((Math.abs(leftStickX) == 0 && Math.abs(leftStickY) == 0) && RobotContainer.getDriverLeftTrigger() <= 0.5) rotationOutput = 0;
//            SmartDashboard.putNumber("PIDTarget", currentAngle);
//            SmartDashboard.putNumber("PIDActual", RobotContainer.drive.getAngleDegrees());
        }else{
            currentAngle = RobotContainer.drive.getAngleDegrees();
            rotationController.reset(currentAngle);
            rotationOutput *= Constants.DRIVE_MAX_ANGULAR_VELOCITY;
        }

        double xVel = -leftStickY * Constants.SWERVE_MAX_VELOCITY;
        double yVel = leftStickX * Constants.SWERVE_MAX_VELOCITY;

//        Translation2d corrections = roomba.calculateMaxVelocities(RobotContainer.drive.getPoseMeters(), xVel, yVel);
        Translation2d corrections = new Translation2d(xVel, yVel);

//        RobotContainer.drive.drive(corrections.getX(), corrections.getY(), rotationOutput, true);
        RobotContainer.drive.driveFieldRelativeCheese(corrections.getX(), corrections.getY(), rotationOutput, RobotContainer.getDriverLeftTrigger() > 0.5 ? 25 : 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.drive.drive(0, 0, 0, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
