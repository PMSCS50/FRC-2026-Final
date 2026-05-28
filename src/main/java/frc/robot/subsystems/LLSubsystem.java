package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

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
    private double omegaRps;
    private LimelightHelpers.PoseEstimate latestEstimate;
    
    public int value = 1;
    private final SwerveDrivePoseEstimator m_poseEstimator = null;

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        new Translation2d(.298, .324),
        new Translation2d(.298, -.324),
        new Translation2d(-.298, .324),
        new Translation2d(-.298, -.324)
    );

    // SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[] {
    //     m_frontLeft.getPosition(),
    //     m_frontRight.getPosition,
    //     m_backLeft.getPosition(),
    //     m_backRight.getPosition
    // }


    private double[] positions;
    public LLSubsystem(CommandSwerveDrivetrain drivetrain, String limelightName) {
        
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        // this.m_poseEstimator = new PoseEstimator(m_kinematics, 
        //                             mGryo.getRotation2d(), 
        //                             m_modulePositions, 
        //                             new Pose2d());

        LimelightHelpers.setPipelineIndex(limelightName, 9);
    }

    @Override
    public void periodic() {
        

        // SmartDashboard.putNumber("Heading sent toLL", headingDeg);
        SmartDashboard.putNumber("Raw Pidgeon Yaw", drivetrain.getPigeon2().getYaw().getValueAsDouble());
        SmartDashboard.putNumber("LL Received Yaw", LimelightHelpers.getIMUData(limelightName).robotYaw);
        SmartDashboard.putNumber("LL IMU mode", LimelightHelpers.getLimelightNTDouble(limelightName, "imumode"));
        
        
        double robotYaw = drivetrain.getPigeon2().getYaw().getValueAsDouble(); //make sure u configured the pigeon2 to be inverted in phoemix tuner x
        var driveState = drivetrain.getState();
        double driveStateYaw = driveState.Pose.getRotation().getDegrees();

        LimelightHelpers.SetRobotOrientation(VisionConstants.limelightName, driveStateYaw, 0, 0, 0, 0, 0);
        PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.limelightName);

        if (LimelightHelpers.validPoseEstimate(llMeasurement)) {
            latestEstimate = llMeasurement;
            //I think that if you use a fixed yaw std of more than 1.5 then it will completely reject the mearuement in drivetrain.
            drivetrain.addVisionMeasurement(
                llMeasurement.pose,
                Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds) 
                // VecBuilder.fill(.05, .05, 3)
            );
            
            SmartDashboard.putNumber("Heading", llMeasurement.pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Tag Count", llMeasurement.tagCount);
            SmartDashboard.putNumber("Avg Tag Distance", llMeasurement.avgTagDist);
            SmartDashboard.putNumber("Distance to Hub", getDistanceToTarget(VisionConstants.getHubPose()));

            
        } else {
            SmartDashboard.putString("Vision Status", "No targets");
            latestEstimate = null;
        }

        Optional<Pose2d> pose = drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds());

        if (pose != null) {

        }
    }






    public boolean hasTarget(int desiredId) {
        return LimelightHelpers.getFiducialID(limelightName) == desiredId;
    }

    // public void updateOdometry() {
    //     m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
    // }
    
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
        return getDistanceToTarget(VisionConstants.getHubPose());
    }

    public double rpmFromDistanceRegression(double distance) {
        double rps = .1322042143 * Math.pow(distance, 4)
        - 1.110063156 * Math.pow(distance, 3) 
        + 3.621489461 * Math.pow(distance, 2)
        + .1849702218 * distance
        + 33.86388054 + 1;
        double rpm = rps * 60;
        return rpm;
    }

    public PoseEstimate getPoseEstimate(PoseEstimate red, PoseEstimate blue) {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? red : blue;
    }



    
}

