package lib.swerve;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

/**
 * Custom version of the wpilib HolonomicDriveController that should work
 * better with paths created with PathPlanner
 */
public class SwervePathController {
    private Translation2d positionError;
    private Translation2d positionTolerance;

    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController rotationController;

    /**
     * Construct a SwervePathController
     *
     * @param xController        PIDController for the robot's X position
     * @param yController        PIDController for the robot's Y position
     * @param rotationController ProfiledPIDController for the robot's rotation
     */
    public SwervePathController(PIDController xController, PIDController yController, ProfiledPIDController rotationController) {
        this.positionError = new Translation2d();
        this.positionTolerance = new Translation2d();

        this.xController = xController;
        this.yController = yController;
        this.rotationController = rotationController;
    }

    /**
     * Calculate if the robot is at its goal position
     *
     * @return Is the robot at its goal position
     */
    public boolean atGoalPose() {
        return Math.abs(positionError.getX()) < positionTolerance.getX() &&
                Math.abs(positionError.getY()) < positionTolerance.getY();
    }

    /**
     * Set the position tolerance of the controller
     *
     * @param tolerance The tolerance of the controller
     */
    public void setPositionTolerance(Translation2d tolerance) {
        this.positionTolerance = tolerance;
    }

    /**
     * Calculate the robot's speeds to match the path
     *
     * @param currentPose Current position of the robot
     * @param goalState   Goal position of the robot
     * @return The calculated speeds and rotation
     */
    public ChassisSpeeds calculate(Pose2d currentPose, SwervePath.State goalState) {
        double xFF = goalState.getXVelocity();
        double yFF = goalState.getYVelocity();
        double rotationFF = rotationController.calculate(currentPose.getRotation().getDegrees(), goalState.getRotation().getDegrees());

        this.positionError = goalState.getPose().relativeTo(currentPose).getTranslation();

        double xFeedback = xController.calculate(currentPose.getX(), goalState.getPose().getX());
        double yFeedback = yController.calculate(currentPose.getY(), goalState.getPose().getY());

        return ChassisSpeeds.fromFieldRelativeSpeeds(xFF + xFeedback, yFF + yFeedback, rotationFF, currentPose.getRotation());
    }
}
