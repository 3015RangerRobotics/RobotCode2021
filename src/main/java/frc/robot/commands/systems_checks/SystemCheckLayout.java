package frc.robot.commands.systems_checks;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import java.util.Map;

public class SystemCheckLayout {

    // Drive motors: position, rotation, and IMU testing
    public static NetworkTableEntry driveFLSR;
    public static NetworkTableEntry driveFLSD;
    public static NetworkTableEntry driveFRSR;
    public static NetworkTableEntry driveFRSD;
    public static NetworkTableEntry driveBLSR;
    public static NetworkTableEntry driveBLSD;
    public static NetworkTableEntry driveBRSR;
    public static NetworkTableEntry driveBRSD;
    public static NetworkTableEntry drivePosition;
    public static NetworkTableEntry driveRotation;
    public static NetworkTableEntry driveIMU;

    public static void init() {
        ShuffleboardLayout testCommands = Shuffleboard.getTab("Systems Check")
                .getLayout("Test Commands", BuiltInLayouts.kList).withSize(1, 3).withPosition(10, 2)
                .withProperties(Map.of("Label position", "HIDDEN"));
        testCommands.add(new TestSwerve());

        ShuffleboardLayout driveValues = Shuffleboard.getTab("Systems Check").getLayout("Drive", BuiltInLayouts.kList)
                .withSize(2, 11).withPosition(0, 0);
        driveFLSR = driveValues.add("FL Rotation", false).getEntry();
        driveFLSD = driveValues.add("FL Drive", false).getEntry();
        driveFRSR = driveValues.add("FR Rotation", false).getEntry();
        driveFRSD = driveValues.add("FR Drive", false).getEntry();
        driveBLSR = driveValues.add("BL Rotation", false).getEntry();
        driveBLSD = driveValues.add("BL Drive", false).getEntry();
        driveBRSR = driveValues.add("BR Rotation", false).getEntry();
        driveBRSD = driveValues.add("BR Drive", false).getEntry();
        drivePosition = driveValues.add("Position", false).getEntry();
        driveRotation = driveValues.add("Rotation", false).getEntry();
        driveIMU = driveValues.add("IMU", false).getEntry();
    }
}
