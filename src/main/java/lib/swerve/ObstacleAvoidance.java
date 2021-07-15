package lib.swerve;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Fancy code and math that should be able to prevent a swerve drive robot from
 * entering specified restricted areas.
 * 
 * This does technically work but errors in odomotry compound and prevent it
 * from being useful beyond a few seconds of driving.
 */
public class ObstacleAvoidance {
    private final double bufferWidth;
    private final double acceleration;
    private final ArrayList<RestrictedArea> restrictedAreas;

    /**
     * Construct an obstacle avoidance object
     *
     * @param bufferWidth            Width of the buffer around the robot that
     *                               should stay out of restricted areas
     * @param assistanceAcceleration The acceleration that will be used to determine
     *                               the stopping distance of the robot
     * @param restrictedAreas        The restricted areas that the robot should
     *                               avoid
     */
    public ObstacleAvoidance(double bufferWidth, double assistanceAcceleration, RestrictedArea... restrictedAreas) {
        this.bufferWidth = bufferWidth;
        this.acceleration = assistanceAcceleration;
        this.restrictedAreas = new ArrayList<>();
        this.restrictedAreas.addAll(Arrays.asList(restrictedAreas));
    }

    /**
     * Calculate the necessary velocity corrections to avoid restricted areas
     *
     * @param currentPose Current pose of the robot
     * @param targetXVel  Target x velocity of the robot
     * @param targetYVel  Target y velocity of the robot
     * @return Adjusted speeds that will assist with avoiding restricted areas
     */
    public Translation2d calculateMaxVelocities(Pose2d currentPose, double targetXVel, double targetYVel) {
        Translation2d centerPos = currentPose.getTranslation();

        double maxVelX = targetXVel;
        double maxVelY = targetYVel;

        for (RestrictedArea area : restrictedAreas) {
            if (targetXVel > 0) {
                maxVelX = Math.min(maxVelX, maxXVelPositive(area, centerPos));
            } else if (targetXVel < 0) {
                maxVelX = Math.max(maxVelX, maxXVelNegative(area, centerPos));
            }

            if (targetYVel > 0) {
                maxVelY = Math.min(maxVelY, maxYVelPositive(area, centerPos));
            } else if (targetYVel < 0) {
                maxVelY = Math.max(maxVelY, maxYVelNegative(area, centerPos));
            }
        }

        return new Translation2d(maxVelX, maxVelY);
    }

    private double maxXVelPositive(RestrictedArea area, Translation2d centerPos) {
        // Check if area is in front of robot
        if (area.greaterThanX(centerPos.getX()) && area.canHitOnXAxis(centerPos, bufferWidth)) {
            double d = area.distanceToXCoord(centerPos.getX() + bufferWidth);
            return Math.sqrt(2 * acceleration * d);
        }
        return Double.POSITIVE_INFINITY;
    }

    private double maxXVelNegative(RestrictedArea area, Translation2d centerPos) {
        // Check if area is behind robot
        if (area.lessThanX(centerPos.getX()) && area.canHitOnXAxis(centerPos, bufferWidth)) {
            double d = area.distanceToXCoord(centerPos.getX() - bufferWidth);
            return -Math.sqrt(2 * acceleration * d);
        }
        return Double.NEGATIVE_INFINITY;
    }

    private double maxYVelPositive(RestrictedArea area, Translation2d centerPos) {
        // Check if area to left? of robot
        if (area.greaterThanY(centerPos.getY()) && area.canHitOnYAxis(centerPos, bufferWidth)) {
            double d = area.distanceToYCoord(centerPos.getY() + bufferWidth);
            return Math.sqrt(2 * acceleration * d);
        }
        return Double.POSITIVE_INFINITY;
    }

    private double maxYVelNegative(RestrictedArea area, Translation2d centerPos) {
        // Check if area is to right? of robot
        if (area.lessThanY(centerPos.getY()) && area.canHitOnYAxis(centerPos, bufferWidth)) {
            double d = area.distanceToYCoord(centerPos.getY() + bufferWidth);
            return -Math.sqrt(2 * acceleration * d);
        }
        return Double.NEGATIVE_INFINITY;
    }

    public static class RestrictedArea {
        private final Translation2d min;
        private final Translation2d max;

        public RestrictedArea(double x1, double y1, double x2, double y2) {
            if (x1 < x2 || y1 < y2) {
                this.min = new Translation2d(x1, y1);
                this.max = new Translation2d(x2, y2);
            } else {
                this.max = new Translation2d(x1, y1);
                this.min = new Translation2d(x2, y2);
            }
        }

        public RestrictedArea(Translation2d min, Translation2d max) {
            this.min = min;
            this.max = max;
        }

        public boolean canHitOnYAxis(Translation2d centerPos, double radius) {
            return max.getX() > (centerPos.getX() - radius) && min.getX() < (centerPos.getX() + radius);
        }

        public boolean canHitOnXAxis(Translation2d centerPos, double radius) {
            return max.getY() > (centerPos.getY() - radius) && min.getY() < (centerPos.getY() + radius);
        }

        public double distanceToXCoord(double x) {
            return Math.max(min.getX() - x, Math.max(0, x - max.getX()));
        }

        public double distanceToYCoord(double y) {
            return Math.max(min.getY() - y, Math.max(0, y - max.getY()));
        }

        public boolean greaterThanX(double x) {
            return min.getX() >= x || max.getX() >= x;
        }

        public boolean lessThanX(double x) {
            return min.getX() <= x || max.getX() <= x;
        }

        public boolean greaterThanY(double y) {
            return min.getY() >= y || max.getY() >= y;
        }

        public boolean lessThanY(double y) {
            return min.getY() <= y || max.getY() <= y;
        }
    }
}
