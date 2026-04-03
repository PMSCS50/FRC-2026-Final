package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import edu.wpi.first.apriltag.AprilTagFieldLayout;


//Limelight VisionSubsystem with MegaTag2. 
//Organized the methods of LimelightHelpers into a more cohesive subsystem.

public class LLSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    private boolean kUseLimelight = false;
    private double omegaRps;
    private LimelightHelpers.PoseEstimate latestEstimate;

    public int value = 1;

    private double[] positions;
    public LLSubsystem(CommandSwerveDrivetrain drivetrain, String limelightName) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;

        LimelightHelpers.setPipelineIndex(limelightName, 9);
    }

    @Override
    public void periodic() {
        LimelightHelpers.SetIMUMode(limelightName, 0);
        var driveState = drivetrain.getState();
        // double headingDeg = driveState.Pose.getRotation().getDegrees();
        double headingDeg = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);




        LimelightHelpers.SetRobotOrientation(limelightName, headingDeg, 0.0, 0.0, 0.0, 0.0, 0.0);
        SmartDashboard.putNumber("Heading sent toLL", headingDeg);
        SmartDashboard.putNumber("Raw Pidgeon Yaw", drivetrain.getPigeon2().getYaw().getValueAsDouble());
        SmartDashboard.putNumber("LL Received Yaw", LimelightHelpers.getIMUData(limelightName).robotYaw);
        SmartDashboard.putNumber("LL IMU mode", LimelightHelpers.getLimelightNTDouble(limelightName, "imumode"));
        SmartDashboard.putNumber("Omega rps", omegaRps);
        

        // LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        // LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        
        PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        PoseEstimate llMeasurementRed = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
        PoseEstimate llMeasurementBlue = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        PoseEstimate llMeasurementMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        
        SmartDashboard.putNumber("tag count", llMeasurement.tagCount);

        SmartDashboard.putString("llmeasurement", llMeasurement.toString());

        if (llMeasurement.tagCount > 0) {
            latestEstimate = getPoseEstimate(llMeasurementRed, llMeasurementBlue);
            // latestEstimate = llMeasurement;
            // latestEstimate = 
            drivetrain.addVisionMeasurement(
                latestEstimate.pose,
                Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds)
            );

            SmartDashboard.putNumber("Field X", llMeasurement.pose.getX());
            SmartDashboard.putNumber("Field Y", llMeasurement.pose.getY());
            SmartDashboard.putNumber("Heading", llMeasurement.pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Tag Count", llMeasurement.tagCount);
            SmartDashboard.putNumber("Avg Tag Distance", llMeasurement.avgTagDist);
            SmartDashboard.putString("Vision Status", "Seeing " + llMeasurement.tagCount + " tag(s)");

            SmartDashboard.putNumber("Distance to Hub", getDistanceToTarget(VisionConstants.getHubPose()));
            SmartDashboard.putNumber("Distance to hub 2", getDistanceToTarget(VisionConstants.getHubPose2()));
            
        } else {
            SmartDashboard.putString("Vision Status", "No targets");
            latestEstimate = null;
        }
    }
    // @Override
    // public void periodic() {
    //     NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
    //     NetworkTableEntry ty = table.getEntry("ty");
    //     double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    //     double limelightMountAngleDegrees = 20; 
    //     // distance from the center of the Limelight lens to the floor
    //     double limelightLensHeightInches = 20.0; 
    //     // distance from the target to the floor    
    //     double goalHeightInches = 60.0; 
    //     double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    //     double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    //     //calculate distance
    //     double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);


    //     var driveState = drivetrain.getState();
    //     double headingDeg = driveState.Pose.getRotation().getDegrees();
    //     double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
    //     LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0.0, 0.0, 0.0, 0.0, 0.0);
    //     var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    //     if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
    //         drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
    //     }

    //     positions = LimelightHelpers.getBotPose_TargetSpace("");

       
    //     SmartDashboard.putNumber("distance from limelight to goal inches", distanceFromLimelightToGoalInches);
    //     SmartDashboard.putNumber("X", getX());
    //     SmartDashboard.putNumber("Y", getY());
    //     SmartDashboard.putNumber("Z", getZ());
    //     SmartDashboard.putNumber("Rotation", getYaw());

    //     if (positions != null && positions.length >= 11) {
    //         SmartDashboard.putNumber("X", getX());
    //         SmartDashboard.putNumber("Y", getY());
    //         SmartDashboard.putNumber("Z", getZ());
    //         SmartDashboard.putNumber("Rotation", getYaw());
    //     } else {
    //         SmartDashboard.putString("Vision Status", "No target detected");
    //     }


        

        
    // }

    public boolean hasTarget(int desiredId) {
        return LimelightHelpers.getFiducialID(limelightName) == desiredId;
    }
    
    // Positions: 0 - X (Left/Right) | 1 - Y (Up/Down) | 2 - Z (Forwards/Backwards)
    // Positions: 3 - Roll | 4 - Pitch | 5 - Yaw
    // Positions: 6 - total latency (cl + tl) | 7 - tag count | 8 - tag span 
    // Positions: 9 - average tag distance from camera | 10 - average tag area (% of image)

        public double getX()        { return latestEstimate != null ? latestEstimate.pose.getX() : 0.0; }
        public double getY()        { return latestEstimate != null ? latestEstimate.pose.getY() : 0.0; }
        public double getYaw()      { return latestEstimate != null ? latestEstimate.pose.getRotation().getDegrees() : 0.0; }
        public double getTagCount() { return latestEstimate != null ? latestEstimate.tagCount : 0; }
        public double getAvgTagDist() { return latestEstimate != null ? latestEstimate.avgTagDist : 0.0; }
        public boolean hasTargets() { return latestEstimate != null && latestEstimate.tagCount > 0; }
    

    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(limelightName, pipeline);
    }

    public double getDistanceToTarget(Pose2d targetPose) {
        if (!hasTargets()) return -1.0;
        return drivetrain.getState().Pose.getTranslation()
            .getDistance(targetPose.getTranslation());
    }


    public double getDistanceToTag(int tagId) {
        if (latestEstimate == null || latestEstimate.rawFiducials == null) return -1.0;
        
        for (RawFiducial fiducial : latestEstimate.rawFiducials) {
            if (fiducial.id == tagId) {
                return fiducial.distToRobot;
            }
        }
        return -1.0;
    }

    public double getBestDistanceToHub() {
        double tagDist = getDistanceToTag(VisionConstants.getMiddleTagId());
        if (tagDist > 0) return tagDist;
        return getDistanceToTarget(VisionConstants.getHubPose2());
    }

    public double rpmFromDistanceRegression(double distance) {
        double rps = .1322042143 * Math.pow(distance, 4)
        - 1.110063156 * Math.pow(distance, 3) 
        + 3.621489461 * Math.pow(distance, 2)
        + .1849702218 * distance
        + 33.86388054;
        double rpm = rps * 60;
        return rpm;
    }

    public PoseEstimate getPoseEstimate(PoseEstimate red, PoseEstimate blue) {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? red : blue;
    }






    



    

    


}
