package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class LimelightWaitUntilOnTarget extends CommandBase {
    public LimelightWaitUntilOnTarget() {
        addRequirements(RobotContainer.limelight);
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
        return Math.abs(RobotContainer.limelight.getTargetAngleX()) <= 1;
    }
}
