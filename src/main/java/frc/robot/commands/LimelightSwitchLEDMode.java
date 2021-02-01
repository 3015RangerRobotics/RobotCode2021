package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;

public class LimelightSwitchLEDMode extends CommandBase {
    /**
     * Creates a new LimelightSwitchPipeline.
     */
    public Limelight.LEDMode mode;

    /**
     * Command to change the Limelight's current LEDMode
     * @param mode The LEDMode to change to
     */
    public LimelightSwitchLEDMode(Limelight.LEDMode mode) {
        addRequirements(RobotContainer.limelight);
        this.mode = mode;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.limelight.setLEDMode(mode);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
