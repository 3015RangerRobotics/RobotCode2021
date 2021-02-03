package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Limelight;

public class CG_Fire extends SequentialCommandGroup {
    public CG_Fire(){
        addCommands(
                new ShooterAutoSpeed(),
                new TurretTurnToTarget(),
                new TurretWaitUntilOnTarget(),
                new ShooterAutoSpeed(false),
                new HoodAutoPosition(false),
                new ShooterWaitUntilPrimed(),
                new ParallelDeadlineGroup(
                        new CarouselShoot()
                        //intake
                )
        );
    }
}
