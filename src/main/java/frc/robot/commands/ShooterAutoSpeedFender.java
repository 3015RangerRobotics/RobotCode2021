package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterAutoSpeedFender extends CommandBase {
    boolean constantUpdate;

    public ShooterAutoSpeedFender() {
        addRequirements(RobotContainer.shooter);
        constantUpdate = true;
    }

    public ShooterAutoSpeedFender(boolean constantUpdate) {
        addRequirements(RobotContainer.shooter);
        this.constantUpdate = constantUpdate;
    }

    @Override
    public void initialize() {
        if(constantUpdate) {
            RobotContainer.shooter.setStateAutoSpeedFender();
        } else {
            RobotContainer.shooter.setStateSpeed(RobotContainer.shooter.getAutoSpeed(true));
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
