package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterWaitUntilPrimed extends CommandBase {
    public ShooterWaitUntilPrimed() {

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.shooter.isPrimed();
    }
}
