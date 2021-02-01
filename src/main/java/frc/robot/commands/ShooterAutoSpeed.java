package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterAutoSpeed extends CommandBase {
    boolean constantUpdate;

    public ShooterAutoSpeed() {
        addRequirements(RobotContainer.shooter);
        constantUpdate = true;
    }

    public ShooterAutoSpeed(boolean constantUpdate) {
        addRequirements(RobotContainer.shooter);
        this.constantUpdate = constantUpdate;
    }

    @Override
    public void initialize() {
        if(constantUpdate) {
            RobotContainer.shooter.setStateAutoSpeed();
        } else {
            RobotContainer.shooter.setStateSpeed(RobotContainer.shooter.getAutoSpeed(false));
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
