package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CG_SearchBRed extends ParallelDeadlineGroup {
    public CG_SearchBRed(){
        super(
                new SequentialCommandGroup(
                        new DriveFollowPath("search_b_red")
                ),
                new CarouselIntake()
        );
    }
}