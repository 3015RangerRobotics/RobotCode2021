package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Carousel;

public class CarouselIntake extends CommandBase {
    public CarouselIntake() {
        addRequirements(RobotContainer.carousel);
    }

    @Override
    public void initialize() {
        if (RobotContainer.carousel.isPaused()) {
            RobotContainer.carousel.setPaused(false);
        } else {
            RobotContainer.carousel.setState(Carousel.State.kFillTo1);
        }
    }

    @Override
    public void execute() {
        if (RobotContainer.carousel.getState() == Carousel.State.kOff){
//            RobotContainer.setDriverRumbleLeft(1);
//            RobotContainer.setDriverRumbleRight(1);
//            RobotContainer.setCoDriverRumbleLeft(1);
//            RobotContainer.setCoDriverRumbleRight(1);
        }
    }


    @Override
    public void end(boolean interrupted) {
//        RobotContainer.setDriverRumbleLeft(0);
//        RobotContainer.setDriverRumbleRight(0);
//        RobotContainer.setCoDriverRumbleLeft(0);
//        RobotContainer.setCoDriverRumbleRight(0);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.carousel.getState() == Carousel.State.kOff;
    }
}
