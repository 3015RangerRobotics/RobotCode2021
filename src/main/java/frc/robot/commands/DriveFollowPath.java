// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import lib.swerve.SwervePath;
import lib.swerve.SwervePathController;

public class DriveFollowPath extends CommandBase {
    Timer timer;
    SwervePath path;
    SwervePathController pathController;
    double lastTime;

    public DriveFollowPath(String pathname) {
        addRequirements(RobotContainer.drive);
        this.timer = new Timer();
        this.path = SwervePath.fromCSV(pathname);

        PIDController posController = new PIDController(Constants.DRIVE_POS_ERROR_CONTROLLER_P, Constants.DRIVE_POS_ERROR_CONTROLLER_I, Constants.DRIVE_POS_ERROR_CONTROLLER_D);
        PIDController headingController = new PIDController(Constants.DRIVE_HEADING_ERROR_CONTROLLER_P, Constants.DRIVE_HEADING_ERROR_CONTROLLER_I, Constants.DRIVE_HEADING_ERROR_CONTROLLER_D);
        ProfiledPIDController rotationController = new ProfiledPIDController(Constants.DRIVE_ROTATION_CONTROLLER_P, Constants.DRIVE_ROTATION_CONTROLLER_I, Constants.DRIVE_ROTATION_CONTROLLER_D,
                new TrapezoidProfile.Constraints(Constants.DRIVE_MAX_ANGULAR_VELOCITY, Constants.DRIVE_MAX_ANGULAR_ACCEL));
        this.pathController = new SwervePathController(posController, headingController, rotationController);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        SwervePath.State initialState = path.getInitialState();
        RobotContainer.drive.resetOdometry(new Pose2d(RobotContainer.drive.getPoseMeters().getTranslation(), initialState.getRotation()));
        pathController.reset(RobotContainer.drive.getPoseMeters());
        lastTime = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double time = timer.get();
        SwervePath.State desiredState = path.sample(time);

        ChassisSpeeds targetSpeeds = pathController.calculate(RobotContainer.drive.getPoseMeters(), desiredState, time - lastTime, timer.hasElapsed(0.1));
        RobotContainer.drive.drive(targetSpeeds);

        lastTime = time;

        // Position Graph
        SmartDashboard.putNumber("PIDTarget", desiredState.getPos());
        SmartDashboard.putNumber("PIDActual", pathController.getTotalDistance());

        // Heading Graph
//        SmartDashboard.putNumber("PIDTarget", desiredState.getHeading().getDegrees());
//        SmartDashboard.putNumber("PIDActual", pathController.getCurrentHeading().getDegrees());

        // Rotation Graph
//        SmartDashboard.putNumber("PIDTarget", desiredState.getRotation().getDegrees());
//        SmartDashboard.putNumber("PIDActual", RobotContainer.drive.getPoseMeters().getRotation().getDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println(timer.get());
        timer.stop();
        RobotContainer.drive.drive(0, 0, 0, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(path.getRuntime());
    }
}
