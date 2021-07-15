package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import lib.WaitForButtonCommand;

public class CG_Powerport extends SequentialCommandGroup {
        public CG_Powerport() {
                super(new ParallelDeadlineGroup(new WaitCommand(3),
                                new LimelightSwitchLEDMode(Limelight.LEDMode.LED_ON), new ShooterSetSpeed(4500),
                                new HoodAutoPosition(), new DriveAutoRotate()),
                                new ParallelDeadlineGroup(new WaitCommand(60.5),
                                                new SequentialCommandGroup(new CarouselShoot(0.1),
                                                                new DriveFollowPathHoldTarget("power_reverse"),
                                                                new ParallelDeadlineGroup(
                                                                                new WaitForButtonCommand(
                                                                                                RobotContainer.driverX),
                                                                                new CarouselIntake()),
                                                                new ParallelDeadlineGroup(
                                                                                new DriveFollowPathHoldTarget("power"),
                                                                                new CarouselIntake()),
                                                                new ParallelDeadlineGroup(
                                                                                new LimelightWaitUntilOnTarget(),
                                                                                new DriveAutoRotate()),
                                                                new CarouselShoot(0.1),
                                                                new DriveFollowPathHoldTarget("power_reverse"),
                                                                new ParallelDeadlineGroup(
                                                                                new WaitForButtonCommand(
                                                                                                RobotContainer.driverX),
                                                                                new CarouselIntake()),
                                                                new ParallelDeadlineGroup(
                                                                                new DriveFollowPathHoldTarget("power"),
                                                                                new CarouselIntake()),
                                                                new ParallelDeadlineGroup(
                                                                                new LimelightWaitUntilOnTarget(),
                                                                                new DriveAutoRotate()),
                                                                new CarouselShoot(0.1),
                                                                new DriveFollowPathHoldTarget("power_reverse"),
                                                                new ParallelDeadlineGroup(
                                                                                new WaitForButtonCommand(
                                                                                                RobotContainer.driverX),
                                                                                new CarouselIntake()),
                                                                new ParallelDeadlineGroup(
                                                                                new DriveFollowPathHoldTarget("power"),
                                                                                new CarouselIntake()),
                                                                new ParallelDeadlineGroup(
                                                                                new LimelightWaitUntilOnTarget(),
                                                                                new DriveAutoRotate()),
                                                                new CarouselShoot(0.1))));
        }
}