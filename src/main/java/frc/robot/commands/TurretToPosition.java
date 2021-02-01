package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurretToPosition extends CommandBase {
    public double angle;

    /**
     * Command to turn the turret to a specific angle
     * @param angle The angle to turn to
     */
    public TurretToPosition(double angle) {
        addRequirements(RobotContainer.turret);
        this.angle = angle;
    }

    @Override
    public void initialize() {
        RobotContainer.turret.setStateToPosition(angle);
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
