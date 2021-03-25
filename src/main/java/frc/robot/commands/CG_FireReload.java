package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Limelight;

public class CG_FireReload extends SequentialCommandGroup {
    public CG_FireReload(){
        addCommands(
                new HoodHold(true),
                new CarouselShootOne(),
                new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new CarouselIntake()
                ),
                new CarouselShootOne(),
                new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new CarouselIntake()
                ),
                new CarouselShootOne(),
                new HoodHold(false)
        );
    }
}
