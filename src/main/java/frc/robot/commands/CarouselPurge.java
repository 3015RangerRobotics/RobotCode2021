package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Carousel;

public class CarouselPurge extends CommandBase {
    public CarouselPurge() {
        addRequirements(RobotContainer.carousel);
    }

    @Override
    public void initialize() {
        RobotContainer.carousel.setState(Carousel.State.kPurgeBall5);
        RobotContainer.carousel.setPaused(false);
        RobotContainer.carousel.setBallCounter(0);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.carousel.setState(Carousel.State.kOff);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
