package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class HoodAutoPosition extends CommandBase {
    boolean constantUpdate;
    double referencePosition = 0;

    public HoodAutoPosition() {
        addRequirements(RobotContainer.hood);
        constantUpdate = true;
    }

    public HoodAutoPosition(boolean constantUpdate) {
        addRequirements(RobotContainer.hood);
        this.constantUpdate = constantUpdate;
    }

    @Override
    public void initialize() {
        referencePosition = RobotContainer.hood.getAutoPosition();
    }

    @Override
    public void execute() {
        if (constantUpdate) RobotContainer.hood.setHoodPosition(RobotContainer.hood.getAutoPosition());
        else RobotContainer.hood.setHoodPosition(referencePosition);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.hood.setHoodPosition(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
