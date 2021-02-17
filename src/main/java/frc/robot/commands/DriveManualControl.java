// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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


//        RobotContainer.drive.setModuleRotation(rightStickX * 180);
//        RobotContainer.drive.setBackLeftRotation(135);
//        RobotContainer.drive.setBackRightRotation(45);
//        RobotContainer.drive.setFrontLeftRotation(45);
//        RobotContainer.drive.setFrontRightRotation(-45);
//        RobotContainer.drive.setModuleDrivePct(leftStickY);
        ChassisSpeeds test = new ChassisSpeeds(leftStickY * Constants.SWERVE_MAX_VELOCITY, 0, Units.degreesToRadians(rightStickX * Constants.DRIVE_MAX_ANGULAR_VELOCITY));
        RobotContainer.drive.drive(test);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
//        RobotContainer.drive.setModuleRotationVoltage(0);
        RobotContainer.drive.setModuleDrivePct(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
