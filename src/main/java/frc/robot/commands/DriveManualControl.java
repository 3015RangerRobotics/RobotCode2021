// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveManualControl extends CommandBase {
    /**
     * Creates a new DriveManualControl.
     */
    public DriveManualControl() {
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

        RobotContainer.drive.setModuleRotationVoltage(12.5 * rightStickX);
        RobotContainer.drive.setModuleDrivePct(leftStickY);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.drive.setModuleRotationVoltage(0);
        RobotContainer.drive.setModuleDrivePct(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
