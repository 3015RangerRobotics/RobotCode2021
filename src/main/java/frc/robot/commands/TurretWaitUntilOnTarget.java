package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurretWaitUntilOnTarget extends CommandBase {
    Timer timer = new Timer();

    public TurretWaitUntilOnTarget() {

    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.turret.isOnTarget() && timer.hasElapsed(0.1);
    }
}
