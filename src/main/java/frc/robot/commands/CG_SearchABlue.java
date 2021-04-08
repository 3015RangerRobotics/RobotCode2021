package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CG_SearchABlue extends ParallelDeadlineGroup {
    public CG_SearchABlue(){
        super(
                new SequentialCommandGroup(
//                        new WaitCommand(0.125),
                        new DriveFollowPath("search_a_blue_test")
                ),
                new CarouselIntake()
//                new SequentialCommandGroup(
//                        new HoodSetPosition(20),
//                        new WaitCommand(.5),
//                        new HoodSetPosition(0)
//                )
        );
    }
}