package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveAutoRotate extends CommandBase {
    /**
     * Creates a new DriveTurnToTarget
     * */

    PIDController rotationController;
    public DriveAutoRotate() {
        addRequirements(RobotContainer.drive);
        rotationController = new PIDController(Constants.DRIVE_TARGETING_CONTROLLER_P, Constants.DRIVE_TARGETING_CONTROLLER_I, Constants.DRIVE_TARGETING_CONTROLLER_D);
        rotationController.setTolerance(0.5);
    }

    @Override
    public void initialize() {
//        rotationController.reset(-RobotContainer.limelight.getTargetAngleX());
    }

    @Override
    public void execute() {
        double leftStickY = RobotContainer.getDriverLeftStickY();
        double leftStickX = RobotContainer.getDriverLeftStickX();
        double rightStickX = RobotContainer.getDriverRightStickX() * 0.75;

        SmartDashboard.putNumber("Targeting Error", rotationController.getPositionError());

//        if(Math.abs(rotationController.getPositionError()) <= Constants.DRIVE_TARGETING_I_ZONE){
//            rotationController.setI(Constants.DRIVE_TARGETING_CONTROLLER_I);
//        }else{
//            rotationController.setI(0);
//        }

        double output = rotationController.calculate(-RobotContainer.limelight.getTargetAngleX(), 0);
        SmartDashboard.putNumber("Targeting Output", output);
        if(!rotationController.atSetpoint() && leftStickX == 0 && leftStickY == 0) {
            if(output < 0) output = Math.min(-Constants.DRIVE_ROTATION_MIN_VELOCITY, output);
            else output = Math.max(Constants.DRIVE_ROTATION_MIN_VELOCITY, output);
        }
        if(!RobotContainer.limelight.hasTarget()){
            output = rightStickX * Constants.DRIVE_MAX_ANGULAR_VELOCITY;
        }
//        System.out.println(output);
        RobotContainer.drive.drive(-leftStickY * Constants.SWERVE_MAX_VELOCITY, leftStickX * Constants.SWERVE_MAX_VELOCITY, output, true);
    }

    @Override
    public void end(boolean interrupted) {
//        RobotContainer.drive.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
