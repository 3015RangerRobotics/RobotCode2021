package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurretTurnToTargetHold extends CommandBase {
    public TurretTurnToTargetHold() {
        addRequirements(RobotContainer.turret);
    }

    @Override
    public void initialize() {
        RobotContainer.turret.setStateTurnHold();
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