package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CG_SearchARed extends ParallelCommandGroup {
    public CG_SearchARed(){
        addCommands(
               new CarouselIntake(),
                new SequentialCommandGroup(
                        new WaitCommand(0.125),
                        new DriveFollowPath("search_a_red")
                )
        );
    }
}