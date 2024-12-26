// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.steelhawks.lib;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {


    private boolean isFlashing = false;
    /** Creates a new Limelight. */
    private String limelightName;
    private static ArrayList<Limelight> limelightArray = new ArrayList<>();

    public Limelight(String limelightName) {
        this.limelightName = limelightName;
        limelightArray.add(this); //adds limelight object's memory address

    }

    // //For debugging and startup purposes
    public void flashLimelight() {

        if(isFlashing) return;

        new Thread(() -> {
            isFlashing = true;
            LimelightHelpers.setLEDMode_ForceBlink(this.getLimelightName());
            System.out.println(this.getLimelightName() + " IS WORKING");

            try {
                Thread.sleep(1000);
            }
            catch (Exception e) {
                e.printStackTrace();
            }
            LimelightHelpers.setLEDMode_ForceOff(this.getLimelightName());
            isFlashing = false;
        }).start();

    }

    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(this.limelightName, pipeline);
    }

    public static ArrayList<Limelight> getLimelightsInUse() {
        return limelightArray;
    }

    public String getLimelightName() {
        return this.limelightName;
    }

    public int getPipeline() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(this.limelightName);
    }

    public double getLimelightLatency() {
        return Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline(this.limelightName) / 1000) - (LimelightHelpers.getLatency_Capture(this.limelightName) / 1000);
    }

    public double getActualLLDelay() {
        return (LimelightHelpers.getLatency_Pipeline(this.limelightName) / 1000) + (LimelightHelpers.getLatency_Capture(this.limelightName) / 1000);
    }

    public double getTagArea() {
        return LimelightHelpers.getTA(this.limelightName);
    }

    public int getNumberOfTagsInView() {
        return LimelightHelpers.getLatestResults(this.limelightName).targets_Fiducials.length;
        // this was return LimelightHelpers.getLatestResults(this.limelightName).targetingResults.targets_Fiducials.length;
    }

    public Pose2d getVisionPredictedRobotPose() {
        //   LimelightHelpers.getLatestResults(this.limelightName);
        if (LimelightHelpers.getTV(this.limelightName)) {
            return LimelightHelpers.getBotPose2d_wpiBlue(this.limelightName);
        }

        return null;


    }

    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber(this.limelightName + " Number of Tags in View", this.getNumberOfTagsInView());
        SmartDashboard.putNumber(this.limelightName + " Latency", this.getLimelightLatency());
        SmartDashboard.putNumber(this.limelightName + " current pipeline", this.getPipeline());
        SmartDashboard.putString("Limelights in Use", getLimelightsInUse().toString());

    }
}