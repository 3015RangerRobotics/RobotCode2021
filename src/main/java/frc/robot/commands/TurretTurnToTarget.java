package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurretTurnToTarget extends CommandBase {
    private double pos;

    public TurretTurnToTarget() {
        addRequirements(RobotContainer.turret);
    }

    @Override
    public void initialize() {
        pos = RobotContainer.turret.getPosition() + RobotContainer.limelight.getTargetAngleX();
        RobotContainer.turret.setStateToPosition(pos);
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