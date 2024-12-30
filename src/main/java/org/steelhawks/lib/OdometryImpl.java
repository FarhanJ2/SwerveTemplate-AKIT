package org.steelhawks.lib;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;
import org.steelhawks.RobotContainer;

public final class OdometryImpl {

    private static SwerveDrivePoseEstimator mPoseEstimator;

    public static void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
        mPoseEstimator = poseEstimator;
    }

    public static double getDistance(Pose2d target) {
        return RobotContainer.s_Swerve.getPose().getTranslation().getDistance(target.getTranslation());
    }

    public static double getVisionPoseError(Limelight limelight) {
        if (limelight == null) return -1;
        Pose2d predictedPose = limelight.getVisionPredictedRobotPose();
        if (predictedPose != null) {
            return Math.abs(mPoseEstimator.getEstimatedPosition().getTranslation().getDistance(predictedPose.getTranslation()));
        }
        return -1;
    }

    public static boolean isValidVisionMeasurement(Limelight limelight) {
        if (limelight == null) return false;
        Pose2d predictedPose = limelight.getVisionPredictedRobotPose();
        if (predictedPose != null && (predictedPose.getX() != 0 && predictedPose.getY() != 0)) {
            return limelight.getTagArea() > Constants.LimelightConstants.MIN_AREA_OF_TAG;
        }
        return false;
    }

    public static Pose2d getVisionMeasurementWithoutYaw(Limelight limelight) {
        if (limelight == null) return null;
        //added variable for predicted pose instead of calling function directly
        Pose2d predictedPose = limelight.getVisionPredictedRobotPose();
        if (isValidVisionMeasurement(limelight) && predictedPose != null) {
            return new Pose2d(predictedPose.getTranslation(), RobotContainer.s_Swerve.getRotation());
        }
        return null;
    }


    public static Pose2d getVisionMeasurement(Limelight limelight) {
        if (limelight == null) return null;
        Pose2d predictedPose = limelight.getVisionPredictedRobotPose();
        if (isValidVisionMeasurement(limelight) && predictedPose != null) {
            return predictedPose;
        }
        return null;
    }

    /** Angle offset should be passed in as degrees */
    public static Vector<N3> createStdDevs(double n1, double n2, double angleOffset) {
        return VecBuilder.fill(n1, n2, Units.degreesToRadians(angleOffset));
    }

    /** Returns standard deviations using tag area; there MUST be multiple tags in sight */
    private static Vector<N3> getCalculatedStdDevsFromDistanceMultipleTags(double area) {
        double xyStdDev = 0.3 / Math.pow(area, 2);
        double thetaStdDev = 99999 / area;

        return createStdDevs(xyStdDev, xyStdDev, thetaStdDev);
    }

    public static Vector<N3> getCalculatedStdDevs(Limelight limelight) {
        double error = getVisionPoseError(limelight);

        if (limelight.getNumberOfTagsInView() >= 2) {
            return getCalculatedStdDevsFromDistanceMultipleTags(limelight.getTagArea());
        }

        // TODO create a formula rather than a bunch of if statements
        else if (limelight.getNumberOfTagsInView() == 1) {

            //one tag with large area but larger pose error
            if (limelight.getTagArea() > 0.6 && error < 0.5) {
                return createStdDevs(3, 3, 99999);
            }

            //one tag with small area but smaller pose error
            else if (limelight.getTagArea() > 0.1 && error < 0.3) {
                return createStdDevs(4, 4, 99999);
            }
        }

        // Return default values if nothing is true
        return createStdDevs(Constants.PoseConfig.VISION_STD_DEV_X, Constants.PoseConfig.VISION_STD_DEV_Y, Constants.PoseConfig.VISION_STD_DEV_THETA);
    }

    static int counter = 0;
    public static void log() {
        counter = (counter + 1) % 1000;

        if (counter % 5 == 0) {
            //  SmartDashboard.putNumber("Vision Pose Error Limelight Front", getVisionPoseError(RobotContainer.s_Swerve.limelightShooter));
            //  SmartDashboard.putNumber("Vision Pose Error Limelight Back", getVisionPoseError(RobotContainer.s_Swerve.limelightArm));
        }
    }
}
