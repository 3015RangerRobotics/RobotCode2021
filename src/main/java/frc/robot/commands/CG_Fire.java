package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Limelight;

public class CG_Fire extends SequentialCommandGroup {
    public CG_Fire(){
        addCommands(
                new ShooterSetSpeed(7000),
                new HoodAutoPosition(true),
//                new HoodSetPosition(29),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new ShooterWaitUntilPrimed(),
                                new CarouselShoot(.5)
                        ),
                        new DriveAutoRotate()
                )
        );
    }
}
