package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveWithGamepadOld extends CommandBase {
    public DriveWithGamepadOld() {
        addRequirements(RobotContainer.driveOld);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double driveValue = -RobotContainer.getDriverLeftStickY();
        double turnValue;

        if(Math.abs(RobotContainer.getDriverRightStickX()) > Math.abs(RobotContainer.getDriverLeftStickX())){
            turnValue = RobotContainer.getDriverRightStickX();
        }else{
            turnValue = RobotContainer.getDriverLeftStickX();
        }

        RobotContainer.driveOld.arcadeDrive(driveValue, turnValue, true);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.driveOld.setMotorOutputs(ControlMode.PercentOutput, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}