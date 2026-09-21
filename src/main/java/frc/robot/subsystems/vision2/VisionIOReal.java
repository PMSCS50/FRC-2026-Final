package frc.robot.subsystems.vision2;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.target.AprilTagFiducial;
import limelight.networktables.LimelightResults;

import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class VisionIOReal implements VisionIO {

    private Pose2d lastGoodPose = null;

    private final String cameraName;
    private final Limelight limelight;
    private final LimelightPoseEstimator poseEstimator;

    /**
     * Real camera IO with configurable mounting.
     *
     * @param cameraName    Limelight name
     */
    public VisionIOReal(String cameraName, Transform3d robotToCamera) {
        this.cameraName = cameraName;
        limelight = new Limelight(cameraName);
        limelight.getSettings()
                .withCameraOffset(new Pose3d(
                    robotToCamera.getX(),
                    robotToCamera.getY(),
                    robotToCamera.getZ(),
                    robotToCamera.getRotation()
                ));

        poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);

    }

    public VisionIOReal(String cameraName) {
        this(cameraName, Transform3d.kZero);
    }

    // *Called by Vision each loop to seed LL orientation.
    public void setRobotYaw(double yawDegrees) {
        limelight.getSettings()
            .withRobotOrientation(
                new Orientation3d(
                    new Rotation3d(0, 0, yawDegrees * 180 / Math.PI),
                    new AngularVelocity3d(
                        DegreesPerSecond.of(0),
                        DegreesPerSecond.of(0),
                        DegreesPerSecond.of(0))))
        .save();
    }

    // *Update IO
    @Override
    public void updateInputs(VisionIOInputs inputs) {

        inputs.hasTarget = limelight.getLatestResults().get().valid;

        if (!inputs.hasTarget) {
            clear(inputs);
            return;
        }

        LimelightResults results = limelight.getLatestResults().get();
        AprilTagFiducial[] fiducials = results.targets_Fiducials;

        int tagCount = fiducials.length;

        if (tagCount < 1) {
            clear(inputs);
            return;
        }

        int[] ids = new int[tagCount];
        Pose2d[] poses = new Pose2d[tagCount];

        for (int i = 0; i < tagCount; i++) {
            ids[i] = (int) fiducials[i].fiducialID;

            poses[i] = fiducials[i].getRobotPose_TargetSpace2D();
        }

        inputs.visibleTagIds   = ids;
        inputs.visibleTagPoses = poses;

        PoseEstimate pe = poseEstimator.getPoseEstimate().get();
        Pose2d pose = pe.pose.toPose2d();
        inputs.hasEstimatedPose = pe.hasData;

        boolean notInFieldArea = pose.getX() < 0 || pose.getX() > Constants.FIELD_MAX_X || 
                              pose.getY() < 0 || pose.getY() > Constants.FIELD_MAX_Y;

        double age = Timer.getFPGATimestamp() - pe.timestampSeconds;
        boolean old = age > 0.25;

        if (pe.getMinTagAmbiguity() > 0.3 || !inputs.hasEstimatedPose || notInFieldArea || old) {
            clearPose(inputs);
            return;
        }

        // |Save last good pose
        lastGoodPose = pose;

        inputs.estimatedPose          = pose;
        inputs.estimatedPoseTimestamp = pe.timestampSeconds;
        inputs.numTagsUsed            = pe.tagCount;

        inputs.ambiguity[0] = pe.getMinTagAmbiguity();
        inputs.ambiguity[1] = pe.getAvgTagAmbiguity();
        inputs.ambiguity[2] = pe.getMaxTagAmbiguity();

        inputs.stdDevs[0] = results.stdev_mt2[0];
        inputs.stdDevs[1] = results.stdev_mt2[1];
        inputs.stdDevs[2] = results.stdev_mt2[5];

        inputs.targetId = ids[0]; // best target = first fiducial
    }

    // private Matrix<N3, N1> calculateStdDevs(PoseEstimate pe) {
    //     if (!pe.hasData) {
    //         return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    //     }

    //     double avgDist = pe.avgTagDist;
    //     double avgAmbiguity = pe.getAvgTagAmbiguity();
    //     int tagCount = pe.tagCount;

    //     double xyStdDev = 20 * (0.05 + (0.08 * Math.pow(avgDist, 2) / tagCount)) * avgAmbiguity;

    //     return VecBuilder.fill(xyStdDev, xyStdDev, Double.MAX_VALUE);
    // }

    // *IO clearing helpers
    private void clear(VisionIOInputs inputs) {
        inputs.hasTarget        = false;
        inputs.targetId         = -1;
        inputs.visibleTagIds    = new int[0];
        inputs.visibleTagPoses  = new Pose2d[0];
        clearPose(inputs);
    }

    private void clearPose(VisionIOInputs inputs) {
        inputs.hasEstimatedPose = false;

        if (lastGoodPose != null) {
            inputs.estimatedPose = lastGoodPose;   // fallback to last good pose
        } else {
            inputs.estimatedPose = new Pose2d();   // no pose yet
        }

        inputs.estimatedPoseTimestamp = 0.0;       // mark as fallback
        inputs.numTagsUsed = 0;
    }

    @Override
    public String getName() {
        return cameraName;
    }
}
