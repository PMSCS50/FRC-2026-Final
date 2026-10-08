package frc.robot.subsystems.vision2;

import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.target.AprilTagFiducial;
import limelight.networktables.LimelightResults;

import limelight.networktables.Orientation3d;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.util.misc.GeometryUtil;

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
                .withCameraOffset(GeometryUtil.toPose3d(robotToCamera));

        poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);

    }

    public VisionIOReal(String cameraName) {
        this(cameraName, Transform3d.kZero);
    }

    // *Called by Vision each loop to seed LL orientation.
    public void setRobotOrientation(Orientation3d orientation) {
        limelight.getSettings()
            .withRobotOrientation(orientation)
            .save();
    }
    
    // *Update IO
    @Override
    public void updateInputs(VisionIOInputs inputs) {

        Optional<LimelightResults> llresults = limelight.getLatestResults();

        if (llresults.isEmpty()) {
            clear(inputs);
            return;
        }

        inputs.hasTarget = llresults.get().valid;

        if (!inputs.hasTarget) {
            clear(inputs);
            return;
        }

        LimelightResults results = llresults.get();
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

        poseEstimator.getPoseEstimate().ifPresentOrElse(
            (pe) -> {
                Pose2d pose = pe.pose.toPose2d();
                inputs.hasEstimatedPose = pe.hasData;

                boolean notInFieldArea = pose.getX() < 0 || pose.getX() > Constants.FIELD_MAX_X || 
                                        pose.getY() < 0 || pose.getY() > Constants.FIELD_MAX_Y;

                double age = Timer.getFPGATimestamp() - pe.timestampSeconds;
                boolean old = age > 0.25;

                if (pe.getMinTagAmbiguity() > 0.3 || pe.rawFiducials == null ||
                pe.rawFiducials.length == 0 || notInFieldArea || old) {
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

                inputs.targetId = ids[0]; // best target = first fiducial
            },
            () -> {
                clearPose(inputs);
                return;
            }
        );
    }

    // private double[] calculateStdDevs(PoseEstimate pe) {
    //     if (!pe.hasData) {
    //         return new double[] {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
    //     }

    //     double avgDist = pe.avgTagDist;
    //     double avgAmbiguity = pe.getAvgTagAmbiguity();
    //     int tagCount = pe.tagCount;

    //     double xyStdDev = 3 * (0.05 + (0.08 * Math.pow(avgDist, 2) / tagCount)) * avgAmbiguity;

    //     return new double[] {xyStdDev, xyStdDev, Double.MAX_VALUE};
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
