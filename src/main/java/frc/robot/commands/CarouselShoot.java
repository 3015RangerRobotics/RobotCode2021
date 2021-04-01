package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Carousel.State;

public class CarouselShoot extends CommandBase {
    private double delay;
    private boolean fender;
    Timer timer = new Timer();

    public CarouselShoot() {
        addRequirements(RobotContainer.carousel);
        this.delay = 0;
        this.fender = false;
    }

    public CarouselShoot(double delay) {
        addRequirements(RobotContainer.carousel);
        this.delay = delay;
        this.fender = false;
    }

    public CarouselShoot(double delay, boolean fender) {
        addRequirements(RobotContainer.carousel);
        this.delay = delay;
        this.fender = fender;
    }

    @Override
    public void initialize() {
        RobotContainer.carousel.setPaused(false);
        RobotContainer.carousel.setState(State.kShootBall1);//start by shooting first ball
        if(!fender) RobotContainer.shooter.selectProfileSlot(1);
        timer.reset();
        timer.start();
        RobotContainer.carousel.setBallCounter(0);
        if(RobotContainer.jankTimer.get() > 60 || RobotContainer.jankTimer.get() == 0){
            RobotContainer.jankTimer.reset();
            RobotContainer.jankTimer.start();
        }
    }

    @Override
    public void execute() {
        if (RobotContainer.carousel.getState() == State.kShootBall1 && !RobotContainer.carousel.isBall1Present() && timer.hasElapsed(delay)) {
            //If the first ball is no longer on the first switch and currently the kShootball1 state is being executed, fire ball 2.
            RobotContainer.carousel.setState(State.kShootBall2);
            timer.reset();
            timer.start();
        } else if (RobotContainer.carousel.getState() == State.kShootBall2 && !RobotContainer.carousel.isBall2Present() && timer.hasElapsed(delay)) {
            //If currently the kShootball2 state is being executed and the second ball is no longer on the second switch, fire ball 3
            RobotContainer.carousel.setState(State.kShootBall3);
            timer.reset();
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.carousel.setState(State.kOff);
        RobotContainer.shooter.selectProfileSlot(0);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.carousel.getState() == State.kShootBall3 && timer.hasElapsed(0.5);
    }
}
