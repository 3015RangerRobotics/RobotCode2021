package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CG_SearchBBlue extends ParallelDeadlineGroup {
    public CG_SearchBBlue(){
        super(
                new SequentialCommandGroup(
                        new WaitCommand(0.25),
                        new DriveFollowPath("search_b_blue")
                ),
                new CarouselIntake()
        );
    }
}