package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterSetSpeed extends CommandBase {
    double rpm;

    public ShooterSetSpeed(double rpm) {
        addRequirements(RobotContainer.shooter);
        this.rpm = rpm;
    }

    @Override
    public void initialize() {
        RobotContainer.shooter.setStateSpeed(rpm);
    }

    @Override
    public void execute() {
//        System.out.println("shooter," + rpm + "," + RobotContainer.shooter.getRPM());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
