package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Carousel;

public class CarouselDefault extends CommandBase {
    Timer timer = new Timer();

    public CarouselDefault() {
        addRequirements(RobotContainer.carousel);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        Carousel.State currentState = RobotContainer.carousel.getState();
        if (currentState == Carousel.State.kFillTo1 || currentState == Carousel.State.kFillTo2 ||
                currentState == Carousel.State.kFillTo3 || currentState == Carousel.State.kFillTo4 ||
                currentState == Carousel.State.kFillTo5) {
            if (timer.hasElapsed(1.5)) {
                RobotContainer.carousel.setPaused(true);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
