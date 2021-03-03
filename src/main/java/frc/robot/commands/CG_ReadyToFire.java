package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Limelight;

public class CG_ReadyToFire extends SequentialCommandGroup {
    public CG_ReadyToFire(){
        addCommands(
                new LimelightSwitchLEDMode(Limelight.LEDMode.LED_ON),
                new LimelightWaitForTarget(),
//                new ShooterAutoSpeed(),
                new ShooterSetSpeed(7000),
//                new HoodAutoPosition(),
                new HoodSetPosition(26.6)
//                new DriveAutoRotate()
//                new TurretTurnToTargetHold()
        );
    }
}
