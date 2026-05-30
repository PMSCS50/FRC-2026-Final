package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

import java.util.Arrays;
import java.util.HashMap;

// Single Limelight subsystem.
// Uses MegaTag2 for pose estimation and fuses into the drivetrain's Kalman filter.

public class LLSubsystemSingle extends VisionGeneral implements VisionIO {

    private final CommandSwerveDrivetrain drivetrain;
    private final String llCamera;

    private final HashMap<Integer, Transform2d> tagtransforms = new HashMap<>();

    private double omegaRps;

    // Pose of the robot, wrapped in latestEstimate
    private Pose2d estimatedRobotPose;
    private PoseEstimate latestEstimate;

    // Vision standard deviations
    private static final double BASE_XY_STD_DEV     = 0.5;
    private static final double THETA_STD_DEV        = 9999.0;
    private static final double MAX_AMBIGUITY        = 0.9;
    private static final double MAX_LATENCY_SECONDS  = 0.25;
    private static final double MAX_OMEGA_RPS        = 2.0;
    private static final double FIELD_MAX_X          = 16.5;
    private static final double FIELD_MAX_Y          = 8.5;

    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public LLSubsystemSingle(CommandSwerveDrivetrain drivetrain, String llCamera) {
        this.drivetrain = drivetrain;
        this.llCamera   = llCamera;

        LimelightHelpers.setPipelineIndex(llCamera, 9);
        LimelightHelpers.SetIMUMode(llCamera, 4);
    }

    @Override
    public void periodic() {
        var driveState = drivetrain.getState();

        double headingDeg = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation(llCamera, headingDeg, 0, 0, 0, 0, 0);

        PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llCamera);

        // Reset each loop to ensure we don't accidentally use stale data if camera is invalid
        estimatedRobotPose = null;
        latestEstimate     = null;

        boolean camValid = isEstimateValid(llMeasurement);

        if (camValid) {
            Matrix<N3, N1> stdDevs = calculateStdDevs(llMeasurement);
            if (stdDevs != null) {
                drivetrain.addVisionMeasurement(
                    llMeasurement.pose,
                    Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds)//,
                    //stdDevs
                );
            }

            estimatedRobotPose = drivetrain.getState().Pose;
            latestEstimate     = llMeasurement;

            Logger.recordOutput("Vision/PE Odometry PE difference magnitude", Math.hypot(
                llMeasurement.pose.getX() - driveState.Pose.getX(),
                llMeasurement.pose.getY() - driveState.Pose.getY()
            ));
        }

        // Reset tagtransforms every periodic
        tagtransforms.clear();

        LimelightTarget_Fiducial[] allTags = LimelightHelpers.getLatestResults(llCamera).targets_Fiducials;
        if (allTags == null) allTags = new LimelightTarget_Fiducial[0];

        // Tag-Transform HashMap
        for (LimelightTarget_Fiducial fiducial : allTags) {
            Pose2d pose = fiducial.getRobotPose_TargetSpace().toPose2d();
            Transform2d tagToRobot = new Transform2d(
                pose.getX(),
                pose.getY(),
                pose.getRotation()
            );
            tagtransforms.put((int) fiducial.fiducialID, tagToRobot);
        }

        // AdvantageKit Logging
        Logger.recordOutput("Vision/Heading Sent to LL", headingDeg);
        Logger.recordOutput("Vision/Raw Pigeon Yaw",     drivetrain.getPigeon2().getYaw().getValueAsDouble());
        Logger.recordOutput("Vision/ω (rps)",          omegaRps);
        Logger.recordOutput("Vision/Tag Count",          llMeasurement != null ? llMeasurement.tagCount : 0);
        Logger.recordOutput("Vision/Camera Valid",       camValid);

        updateInputs(inputs);
        Logger.processInputs("LoggedVision", inputs);
    }

    // Validation

    private boolean isEstimateValid(PoseEstimate estimate) {
        if (estimate == null || estimate.tagCount == 0) return false;

        double ageSeconds = Utils.fpgaToCurrentTime(0)
                          - Utils.fpgaToCurrentTime(estimate.timestampSeconds);
        if (ageSeconds > MAX_LATENCY_SECONDS) return false;

        if (Math.abs(omegaRps) > MAX_OMEGA_RPS) return false;

        Pose2d pose = estimate.pose;
        if (pose.getX() < 0 || pose.getX() > FIELD_MAX_X) return false;
        if (pose.getY() < 0 || pose.getY() > FIELD_MAX_Y) return false;

        return true;
    }

    // Vision STDs

    private Matrix<N3, N1> calculateStdDevs(PoseEstimate estimate) {
        if (estimate == null || estimate.tagCount == 0) return VecBuilder.fill(9999.0, 9999.0, 9999.0);

        double xyStdDev = BASE_XY_STD_DEV;

        xyStdDev /= (0.35 * estimate.tagCount + 0.65 * estimate.avgTagDist);
        xyStdDev *= Math.pow(estimate.avgTagDist, 2);

        if (estimate.rawFiducials != null) {
            double maxAmbiguity = 0;
            for (RawFiducial tag : estimate.rawFiducials) {
                maxAmbiguity = Math.max(maxAmbiguity, tag.ambiguity);
            }
            if (maxAmbiguity > MAX_AMBIGUITY) return VecBuilder.fill(9999.0, 9999.0, 9999.0);
            xyStdDev *= (1.0 + maxAmbiguity * 2.0);
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, THETA_STD_DEV);
    }

    // Getters

    public Pose2d  getPose()        { return estimatedRobotPose; }
    public double  getX(int i)      { return estimatedRobotPose != null ? estimatedRobotPose.getX() : 0.0; }
    public double  getY(int i)      { return estimatedRobotPose != null ? estimatedRobotPose.getY() : 0.0; }
    public double  getYaw()         { return estimatedRobotPose != null ? estimatedRobotPose.getRotation().getDegrees() : 0.0; }
    public double  getYawRad(int i) { return estimatedRobotPose != null ? estimatedRobotPose.getRotation().getRadians() : 0.0; }
    public int     getTagCount()    { return latestEstimate != null ? latestEstimate.tagCount : 0; }
    public double  getAvgTagDist()  { return latestEstimate != null ? latestEstimate.avgTagDist : 0.0; }
    public double  getAvgTagArea()  { return latestEstimate != null ? latestEstimate.avgTagArea : 0.0; }
    public boolean hasTargets()     { return LimelightHelpers.getTV(llCamera); }

    public double getTagX(int id) {
        return hasTarget(id) ? tagtransforms.get(id).getX() : 0.0;
    }

    public double getTagY(int id) {
        return hasTarget(id) ? tagtransforms.get(id).getY() : 0.0;
    }

    public double getTagYaw(int id) {
        return hasTarget(id) ? tagtransforms.get(id).getRotation().getRadians() : 0.0;
    }

    public Transform2d getTransformToTag(int id) {
        return hasTarget(id) ? tagtransforms.get(id) : Transform2d.kZero;
    }

    public boolean hasTarget(int desiredId) {
        return hasFiducial(llCamera, desiredId);
    }

    private boolean hasFiducial(String cameraName, int id) {
        return LimelightHelpers.getFiducialID(cameraName) == id;
    }

    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(llCamera, pipeline);
    }

    public void setIMUMode(int mode) {
        LimelightHelpers.SetIMUMode(llCamera, mode);
    }

    // Distance utilities

    public double getDistanceToTarget(Pose2d targetPose) {
        if (!hasTargets()) return -1.0;
        return estimatedRobotPose.getTranslation().getDistance(targetPose.getTranslation());
    }

    public double getYawToTarget(Pose2d targetPose) {
        if (!hasTargets()) return -1.0;
        return estimatedRobotPose.minus(targetPose).getRotation().getRadians();
    }

    public double getDistanceToTag(int tagId) {
        if (latestEstimate == null || latestEstimate.rawFiducials == null) return -1.0;
        for (RawFiducial fiducial : latestEstimate.rawFiducials) {
            if (fiducial.id == tagId) return fiducial.distToRobot;
        }
        return -1.0;
    }

    // Returns distance to the closest visible tag
    public double getDistance() {
        if (latestEstimate == null || latestEstimate.rawFiducials == null || latestEstimate.rawFiducials.length == 0) return -1.0;

        RawFiducial closest = null;
        double minDist = Double.MAX_VALUE;

        for (RawFiducial fiducial : latestEstimate.rawFiducials) {
            if (fiducial.distToRobot < minDist) {
                minDist = fiducial.distToRobot;
                closest = fiducial;
            }
        }

        return closest != null ? closest.distToRobot : -1.0;
    }

    // Returns the best available distance to a target — raw tag distance first, pose-based fallback
    public double getBestDistanceToTarget(int tagId, Pose2d targetPose) {
        double tagDist = getDistanceToTag(tagId);
        if (tagDist > 0) return tagDist;
        return getDistanceToTarget(targetPose);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.hasTarget       = hasTargets();
        inputs.targetId        = hasTargets() ? (int) LimelightHelpers.getFiducialID(llCamera) : -1;
        inputs.hasTagTransform = inputs.hasTarget;

        inputs.tagToRobotX    = inputs.hasTagTransform ? getTagX(inputs.targetId) : 0.0;
        inputs.tagToRobotY    = inputs.hasTagTransform ? getTagY(inputs.targetId) : 0.0;
        inputs.tagToRobotZ    = 0.0;
        inputs.tagToRobotRotZ = inputs.hasTagTransform ? getTagYaw(inputs.targetId) : 0.0;

        inputs.visibleTagIds   = tagtransforms.keySet().stream().mapToInt(Integer::intValue).toArray();
        inputs.visibleTagPoses = Arrays.stream(inputs.visibleTagIds)
                                       .filter(tagtransforms::containsKey)
                                       .mapToObj(id -> new Pose2d(getTagX(id), getTagY(id), new Rotation2d(getTagYaw(id))))
                                       .toArray(Pose2d[]::new);

        Logger.recordOutput("Vision/VisibleTagIds",   inputs.visibleTagIds);
        Logger.recordOutput("Vision/VisibleTagPoses", inputs.visibleTagPoses);

        inputs.allTagToRobotX    = Arrays.stream(inputs.visibleTagIds).mapToDouble(this::getTagX).toArray();
        inputs.allTagToRobotY    = Arrays.stream(inputs.visibleTagIds).mapToDouble(this::getTagY).toArray();
        inputs.allTagToRobotZ    = new double[inputs.visibleTagIds.length];
        inputs.allTagToRobotRotZ = Arrays.stream(inputs.visibleTagIds).mapToDouble(this::getTagYaw).toArray();

        inputs.hasEstimatedPose       = estimatedRobotPose != null;
        inputs.estimatedPose          = estimatedRobotPose != null ? estimatedRobotPose : new Pose2d();
        inputs.estimatedPoseTimestamp = latestEstimate != null ? latestEstimate.timestampSeconds : 0.0;
        inputs.numTagsUsed            = latestEstimate != null ? latestEstimate.tagCount : 0;
        inputs.avgTagDistMeters       = latestEstimate != null ? latestEstimate.avgTagDist : 0.0;

        Matrix<N3, N1> stdDevMatrix = calculateStdDevs(latestEstimate);
        inputs.visionStdDevs = new double[]{stdDevMatrix.get(0, 0), stdDevMatrix.get(1, 0), stdDevMatrix.get(2, 0)};
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