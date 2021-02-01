package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterStop extends CommandBase {
    public ShooterStop() {
        addRequirements(RobotContainer.shooter);
    }

    @Override
    public void initialize() {
        RobotContainer.shooter.setStateOff();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
