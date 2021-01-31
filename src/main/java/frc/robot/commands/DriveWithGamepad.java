// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveWithGamepad extends CommandBase {
    /**
     * Creates a new DriveWithGamepad.
     */
    public DriveWithGamepad() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double rightStickX = RobotContainer.getDriverRightStickX();
        double leftStickY = RobotContainer.getDriverLeftStickY();
        double leftStickX = RobotContainer.getDriverLeftStickX();

        RobotContainer.drive.drive(leftStickX * Constants.SWERVE_MAX_VELOCITY, leftStickY * Constants.SWERVE_MAX_VELOCITY, rightStickX * Constants.SWERVE_MAX_ANGULAR_VELOCITY, true);
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
