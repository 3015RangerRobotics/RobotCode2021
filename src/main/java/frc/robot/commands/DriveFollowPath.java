// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
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

    public DriveFollowPath(String pathname) {
        addRequirements(RobotContainer.drive);
        this.timer = new Timer();
        this.path = SwervePath.fromCSV(pathname);

        PIDController xController = new PIDController(Constants.DRIVE_ERROR_CONTROLLER_P, Constants.DRIVE_ERROR_CONTROLLER_I, Constants.DRIVE_ERROR_CONTROLLER_D);
        PIDController yController = new PIDController(Constants.DRIVE_ERROR_CONTROLLER_P, Constants.DRIVE_ERROR_CONTROLLER_I, Constants.DRIVE_ERROR_CONTROLLER_D);
        ProfiledPIDController rotationController = new ProfiledPIDController(Constants.DRIVE_ROTATION_CONTROLLER_P, Constants.DRIVE_ROTATION_CONTROLLER_I, Constants.DRIVE_ROTATION_CONTROLLER_D,
                new TrapezoidProfile.Constraints(Constants.DRIVE_MAX_ANGULAR_VELOCITY, Constants.DRIVE_MAX_ANGULAR_ACCEL));
        this.pathController = new SwervePathController(xController, yController, rotationController);
    }

    public DriveFollowPath(double x, double y, double maxVelocity, double maxAcceleration) {
        addRequirements(RobotContainer.drive);
        this.timer = new Timer();
        this.path = SwervePath.generate1D(x, y, maxVelocity, maxAcceleration);

        PIDController xController = new PIDController(Constants.DRIVE_ERROR_CONTROLLER_P, Constants.DRIVE_ERROR_CONTROLLER_I, Constants.DRIVE_ERROR_CONTROLLER_D);
        PIDController yController = new PIDController(Constants.DRIVE_ERROR_CONTROLLER_P, Constants.DRIVE_ERROR_CONTROLLER_I, Constants.DRIVE_ERROR_CONTROLLER_D);
        ProfiledPIDController rotationController = new ProfiledPIDController(Constants.DRIVE_ROTATION_CONTROLLER_P, Constants.DRIVE_ROTATION_CONTROLLER_I, Constants.DRIVE_ROTATION_CONTROLLER_D,
                new TrapezoidProfile.Constraints(Constants.DRIVE_MAX_ANGULAR_VELOCITY, Constants.DRIVE_MAX_ANGULAR_ACCEL));
        this.pathController = new SwervePathController(xController, yController, rotationController);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        SwervePath.State initialState = path.getInitialState();
//        RobotContainer.drive.setAngle(initialState.getRotation().getDegrees());
        System.out.println(RobotContainer.drive.getAngleDegrees());
        RobotContainer.drive.resetOdometry(new Pose2d(initialState.getPose().getTranslation(), initialState.getRotation()));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
//        System.out.println(path.getRuntime());
        RobotContainer.drive.updateOdometry();
        SwervePath.State desiredState = path.sample(timer.get());
        SmartDashboard.putNumber("PIDTarget", desiredState.getPose().getX());
        SmartDashboard.putNumber("PIDActual", RobotContainer.drive.getPoseMeters().getX());
        ChassisSpeeds targetSpeeds = pathController.calculate(RobotContainer.drive.getPoseMeters(), RobotContainer.drive.getAngleRotation2d(), desiredState);
        RobotContainer.drive.drive(targetSpeeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        RobotContainer.drive.drive(0, 0, 0, false);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(path.getRuntime());
    }
}
