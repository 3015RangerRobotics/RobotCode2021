package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurretToDefaultPosition extends CommandBase {

    public TurretToDefaultPosition() {
        addRequirements(RobotContainer.turret);
    }

    @Override
    public void initialize() {
        RobotContainer.turret.setStateDefault();
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
