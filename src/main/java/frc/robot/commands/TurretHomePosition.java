package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurretHomePosition extends CommandBase {
    // I don't know why this needed the intake or why it needed the intake to be down.
    
//    public TurretHomePosition() {
//        addRequirements(RobotContainer.turret, RobotContainer.intake);
//    }
    
    public TurretHomePosition() {
        addRequirements(RobotContainer.turret);
    }

    @Override
    public void initialize() {
        RobotContainer.turret.setStateHoming();
//        RobotContainer.intake.intakeDown();
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
