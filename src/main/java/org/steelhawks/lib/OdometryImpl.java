// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.steelhawks.lib;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;
import org.steelhawks.Robot;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.swerve.Swerve;

public class OdometryImpl {

    public double getDistance(Pose2d target) {
        return RobotContainer.s_Swerve.getPose().getTranslation().getDistance(target.getTranslation());
    }

    //This is assuming that the robot is directly facing the target object
    public double getTurnAngle(Pose2d target, double robotAngle) {
        double tx = target.getX();
        double ty = target.getY();
        double rx = RobotContainer.s_Swerve.getPose().getX();
        double ry = RobotContainer.s_Swerve.getPose().getY();

        double requestedAngle = Math.atan((ty - ry) / (tx - rx)) * (180/ Math.PI);
        double calculatedAngle = (180 - robotAngle + requestedAngle);

        return ((calculatedAngle + 180) % 360) - 180;
    }

    public double getVisionPoseError(Limelight limelight) {
        if(limelight == null) return -1;
        Pose2d predictedPose = limelight.getVisionPredictedRobotPose();
        if (predictedPose != null) {
            return Math.abs(RobotContainer.s_Swerve.mPoseEstimator.getEstimatedPosition().getTranslation().getDistance(predictedPose.getTranslation()));
        }
        return -1;
    }

    public boolean isValidVisionMeasurement(Limelight limelight) {
        if(limelight == null) return false;
        Pose2d predictedPose = limelight.getVisionPredictedRobotPose();
        if (predictedPose != null && (predictedPose.getX() != 0 && predictedPose.getY() != 0)) {
            return limelight.getTagArea() > Constants.LimelightConstants.minAreaOfTag;
        }
        return false;
    }

    public Pose2d getVisionMeasurementWithoutYaw(Limelight limelight) {
        if(limelight == null) return null;
        //added variable for predicted pose instead of calling function directly
        Pose2d predictedPose = limelight.getVisionPredictedRobotPose();
        if (isValidVisionMeasurement(limelight) && predictedPose != null) {
            return new Pose2d(predictedPose.getTranslation(), RobotContainer.s_Swerve.getRotation());
        }
        return null;
    }


    public Pose2d getVisionMeasurement(Limelight limelight) {
        if(limelight == null) return null;
        Pose2d predictedPose = limelight.getVisionPredictedRobotPose();
        if (isValidVisionMeasurement(limelight) && predictedPose != null) {
            return predictedPose;
        }
        return null;
    }

    /** Angle offset should be passed in as degrees */
    public Vector<N3> createStdDevs(double n1, double n2, double angleOffset) {
        return VecBuilder.fill(n1, n2, Units.degreesToRadians(angleOffset));
    }

    /** Returns standard deviations using tag area; there MUST be multiple tags in sight */
    private Vector<N3> getCalculatedStdDevsFromDistanceMultipleTags(double area) {
        double xyStdDev = 0.3 / Math.pow(area, 2);
        double thetaStdDev = 99999 / area;

        return createStdDevs(xyStdDev, xyStdDev, thetaStdDev);
    }

    public Vector<N3> getCalculatedStdDevs(Limelight limelight) {
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

    public void periodic() {
//        SmartDashboard.putNumber("Vision Pose Error Limelight Front", getVisionPoseError(RobotContainer.s_Swerve.limelightShooter));
//        SmartDashboard.putNumber("Vision Pose Error Limelight Back", getVisionPoseError(RobotContainer.s_Swerve.limelightArm));
    }
}
