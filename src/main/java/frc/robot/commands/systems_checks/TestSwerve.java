package frc.robot.commands.systems_checks;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import static frc.robot.Constants.*;

public class TestSwerve extends CommandBase {

    Timer timer = new Timer();
    boolean motorsChecked = false;
    boolean gyroChecked = false;
    boolean turnChecked = false;
    boolean turnSet = false;
    boolean done = false;
    boolean[] rotations = {false, false, false, false};
    Translation2d startTranslation;
    Translation2d endTranslation;


    public TestSwerve() {
        addRequirements(RobotContainer.drive);
    }

    public void initialize() {
        timer.reset();
        timer.start();
        RobotContainer.drive.resetIMU();
        RobotContainer.drive.resetEncoders();
        motorsChecked = false;
        gyroChecked = false;
        turnChecked = false;
        turnSet = false;
        done = false;
        SystemCheckLayout.driveFLSR.setBoolean(false);
        SystemCheckLayout.driveFLSD.setBoolean(false);
        SystemCheckLayout.driveFRSR.setBoolean(false);
        SystemCheckLayout.driveFRSD.setBoolean(false);
        SystemCheckLayout.driveBLSR.setBoolean(false);
        SystemCheckLayout.driveBLSD.setBoolean(false);
        SystemCheckLayout.driveBRSR.setBoolean(false);
        SystemCheckLayout.driveBRSD.setBoolean(false);
        SystemCheckLayout.driveIMU.setBoolean(false);
        SystemCheckLayout.drivePosition.setBoolean(false);
        SystemCheckLayout.driveRotation.setBoolean(false);
        startTranslation = RobotContainer.drive.getPoseMeters().getTranslation();
    }

    public void execute() {
        if(!timer.hasElapsed(0.2)) {
            RobotContainer.drive.setModuleDrivePct(0.2);
        } else if(!motorsChecked) {
            endTranslation = RobotContainer.drive.getPoseMeters().getTranslation();
            double dist = endTranslation.minus(startTranslation).getNorm();
            SystemCheckLayout.drivePosition.setBoolean(dist >= 0.5);
            SystemCheckLayout.driveFLSD.setBoolean(RobotContainer.pdp.getCurrent(SWERVE_DRIVE_CHANNEL_FL) >= 5);
            SystemCheckLayout.driveFRSD.setBoolean(RobotContainer.pdp.getCurrent(SWERVE_DRIVE_CHANNEL_FR) >= 5);
            SystemCheckLayout.driveBLSD.setBoolean(RobotContainer.pdp.getCurrent(SWERVE_DRIVE_CHANNEL_BL) >= 5);
            SystemCheckLayout.driveBRSD.setBoolean(RobotContainer.pdp.getCurrent(SWERVE_DRIVE_CHANNEL_BR) >= 5);
            motorsChecked = true;
        } else if(!timer.hasElapsed(0.7)) {
            RobotContainer.drive.setModuleDrivePct(0);
        } else if(!timer.hasElapsed(0.95)) {
            RobotContainer.drive.setModuleRotation(90);
        } else if(!timer.hasElapsed(1.05)) {
            rotations[0] = RobotContainer.drive.getFrontLeftRotation() >= 85 && RobotContainer.drive.getFrontLeftRotation() <= 95;
            rotations[1] = RobotContainer.drive.getFrontRightRotation() >= 85 && RobotContainer.drive.getFrontRightRotation() <= 95;
            rotations[2] = RobotContainer.drive.getBackLeftRotation() >= 85 && RobotContainer.drive.getBackLeftRotation() <= 95;
            rotations[3] = RobotContainer.drive.getBackRightRotation() >= 85 && RobotContainer.drive.getBackRightRotation() <= 95;
        } else if(!timer.hasElapsed(1.4)) {
            RobotContainer.drive.setModuleRotation(-90);
        } else if(!turnChecked) {
            rotations[0] &= RobotContainer.drive.getFrontLeftRotation() >= -95 && RobotContainer.drive.getFrontLeftRotation() <= -85;
            rotations[1] &= RobotContainer.drive.getFrontRightRotation() >= -95 && RobotContainer.drive.getFrontRightRotation() <= -85;
            rotations[2] &= RobotContainer.drive.getBackLeftRotation() >= -95 && RobotContainer.drive.getBackLeftRotation() <= -85;
            rotations[3] &= RobotContainer.drive.getBackRightRotation() >= -95 && RobotContainer.drive.getBackRightRotation() <= -85;
            SystemCheckLayout.driveFLSR.setBoolean(rotations[0]);
            SystemCheckLayout.driveFRSR.setBoolean(rotations[1]);
            SystemCheckLayout.driveBLSR.setBoolean(rotations[2]);
            SystemCheckLayout.driveBRSR.setBoolean(rotations[3]);
        } else if(!timer.hasElapsed(1.65)) {
            RobotContainer.drive.setModuleRotation(0);
        } else if(!timer.hasElapsed(2.05)) {
            RobotContainer.drive.setAngle(90);
        } else if(!gyroChecked) {
            SystemCheckLayout.driveIMU.setBoolean(RobotContainer.drive.getAngleDegrees() >= 80);
            gyroChecked = true;
        } else if(!turnChecked) {
            SystemCheckLayout.driveRotation.setBoolean(RobotContainer.drive.getAngleDegrees() >= 85 && RobotContainer.drive.getAngleDegrees() <= 95);
            turnChecked = true;
        } else if(!timer.hasElapsed(2.55)) {
            RobotContainer.drive.setAngle(0);
        } else {
            done = true;
        }
    }

    public void end(boolean interrupted) {
        RobotContainer.drive.setModuleDrivePct(0);
    }

    public boolean isFinished() {
        return done;
    }
}
