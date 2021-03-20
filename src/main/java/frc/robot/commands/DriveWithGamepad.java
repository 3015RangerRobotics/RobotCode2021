// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveWithGamepad extends CommandBase {
    ProfiledPIDController rotationController;
    double currentAngle;

    /**
     * Creates a new DriveWithGamepad.
     */
    public DriveWithGamepad() {
        addRequirements(RobotContainer.drive);
        rotationController = new ProfiledPIDController(Constants.DRIVE_ROTATION_CONTROLLER_P, Constants.DRIVE_ROTATION_CONTROLLER_I, Constants.DRIVE_ROTATION_CONTROLLER_D,
                new TrapezoidProfile.Constraints(Constants.DRIVE_MAX_ANGULAR_VELOCITY, Constants.DRIVE_MAX_ANGULAR_ACCEL));
        rotationController.enableContinuousInput(-180, 180);
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
        double rightStickX = RobotContainer.getDriverRightStickX() ;
        double leftStickY = RobotContainer.getDriverLeftStickY();
        double leftStickX = RobotContainer.getDriverLeftStickX();

        double rotationOutput = rightStickX;
        if(Math.abs(rotationOutput) == 0){
            rotationOutput = rotationController.calculate(RobotContainer.drive.getAngleDegrees(), currentAngle);
            if((Math.abs(leftStickX) == 0 && Math.abs(leftStickY) == 0)) rotationOutput = 0;
//            SmartDashboard.putNumber("PIDTarget", currentAngle);
//            SmartDashboard.putNumber("PIDActual", RobotContainer.drive.getAngleDegrees());
        }else{
            currentAngle = RobotContainer.drive.getAngleDegrees();
            rotationController.reset(currentAngle);
            rotationOutput *= Constants.DRIVE_MAX_ANGULAR_VELOCITY;
        }

        RobotContainer.drive.drive(-leftStickY * Constants.SWERVE_MAX_VELOCITY, leftStickX * Constants.SWERVE_MAX_VELOCITY, rotationOutput, true);
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
