package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class HoodAutoPosition extends CommandBase {
    boolean constantUpdate;

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
        if(constantUpdate){
            RobotContainer.hood.setStateAutoPosition();
        }else{
            RobotContainer.hood.setStatePosition(RobotContainer.hood.getAutoPosition());
        }
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
