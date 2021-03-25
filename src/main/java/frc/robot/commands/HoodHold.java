package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class HoodHold extends CommandBase {
    boolean hold;

    public HoodHold(boolean hold) {
        this.hold = hold;
    }

    @Override
    public void initialize() {
        RobotContainer.hood.setHoldAutoPos(hold);
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
