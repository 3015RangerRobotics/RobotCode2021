package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CG_Bounce extends SequentialCommandGroup {
    public CG_Bounce(){
        addCommands(
                new DriveFollowPath("bounce_1"),
                new DriveFollowPath("bounce_2"),
                new DriveFollowPath("bounce_3"),
                new DriveFollowPath("bounce_4")
        );
    }
}