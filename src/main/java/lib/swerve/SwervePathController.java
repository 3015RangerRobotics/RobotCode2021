package lib.swerve;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.RobotContainer;

public class SwervePathController {
    private final PIDController posErrorController;
    private final PIDController headingErrorController;
    private final ProfiledPIDController rotationController;

    private Translation2d lastPosition;
    private double totalDistance;
    private Rotation2d currentHeading;

    /**
     * Construct a SwervePathController
     *
     * @param posErrorController     PIDController for the robot's position
     * @param headingErrorController PIDController for the robot's heading
     * @param rotationController     ProfiledPIDController for the robot's rotation
     */
    public SwervePathController(PIDController posErrorController, PIDController headingErrorController, ProfiledPIDController rotationController) {
        this.posErrorController = posErrorController;
        this.headingErrorController = headingErrorController;
        this.rotationController = rotationController;
        this.lastPosition = new Translation2d();
        this.totalDistance = 0;
        this.currentHeading = new Rotation2d(0);
    }

    /**
     * Reset the state of the path controller
     *
     * @param currentPose The current pose of the robot
     */
    public void reset(Pose2d currentPose) {
        this.posErrorController.reset();
        this.headingErrorController.reset();
        this.rotationController.reset(currentPose.getRotation().getDegrees());
        this.lastPosition = currentPose.getTranslation();
        this.totalDistance = 0;
        this.currentHeading = new Rotation2d(0);
    }

    public double getTotalDistance(){
        return this.totalDistance;
    }

    public Rotation2d getCurrentHeading(){
        return this.currentHeading;
    }

    /**
     * Calculate the robot's speeds to match the path
     *
     * @param currentPose     Current pose of the robot
     * @param goalState       Goal state of the robot
     * @return The calculated speeds and rotation
     */
    public ChassisSpeeds calculate(Pose2d currentPose, SwervePath.State goalState, double deltaTime) {
        Translation2d currentPos = currentPose.getTranslation();
        Rotation2d currentRotation = currentPose.getRotation();

        totalDistance += lastPosition.getDistance(currentPos);
        double xV = (currentPos.getX() - lastPosition.getX()) / deltaTime;
        double yV = (currentPos.getY() - lastPosition.getY()) / deltaTime;
        this.currentHeading = new Rotation2d(Math.atan2(yV, xV));

        double vel = goalState.getVelocity();
        Rotation2d heading = goalState.getHeading();
        double rotSpeed = rotationController.calculate(currentRotation.getDegrees(), goalState.getRotation().getDegrees());

        vel += posErrorController.calculate(totalDistance, goalState.getPos());
        heading.plus(Rotation2d.fromDegrees(headingErrorController.calculate(this.currentHeading.getDegrees(), goalState.getHeading().getDegrees())));

        double xVel = vel * heading.getCos();
        double yVel = vel * heading.getSin();

        this.lastPosition = currentPos;

        return ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, Units.degreesToRadians(rotSpeed), currentRotation);
    }
}
