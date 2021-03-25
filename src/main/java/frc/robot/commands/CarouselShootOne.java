package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Carousel.State;

public class CarouselShootOne extends CommandBase {
    Timer timer = new Timer();

    public CarouselShootOne() {
        addRequirements(RobotContainer.carousel);
    }

    @Override
    public void initialize() {
        RobotContainer.carousel.setPaused(false);
        RobotContainer.carousel.setState(State.kShootBall1);//start by shooting first ball
        timer.reset();
        timer.start();
        RobotContainer.carousel.setBallCounter(0);
    }

    @Override
    public void execute() {
        if (RobotContainer.carousel.getState() == State.kShootBall1 && !RobotContainer.carousel.isBall1Present() && timer.hasElapsed(0.25)) {
            RobotContainer.carousel.setState(State.kOff);
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.carousel.setState(State.kOff);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.carousel.getState() == State.kOff && timer.hasElapsed(0.25);
    }
}
