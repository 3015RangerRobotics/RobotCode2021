package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CG_SearchABlue extends ParallelDeadlineGroup {
    public CG_SearchABlue() {
        super(
            new SequentialCommandGroup(
                new DriveFollowPath("search_a_blue_test")
            ), 
            new CarouselIntake()
        );
    }
}