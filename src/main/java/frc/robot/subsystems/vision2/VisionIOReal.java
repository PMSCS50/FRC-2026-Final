package frc.robot.subsystems.vision2;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class VisionIOReal implements VisionIO {

    private final String name;

    private Pose2d lastGoodPose = null;

    /**
     * Real camera IO with configurable mounting.
     *
     * @param cameraName    Limelight name
     */
    public VisionIOReal(String cameraName, Transform3d robotToCamera) {
        this.name = cameraName;
        LimelightHelpers.setCameraPose_RobotSpace(
            name,
            robotToCamera.getX(),
            robotToCamera.getY(),
            robotToCamera.getZ(),
            robotToCamera.getRotation().getX(),  // roll (deg)
            robotToCamera.getRotation().getY(),  // pitch (deg)
            robotToCamera.getRotation().getZ()   // yaw (deg)
        );
    }

    public VisionIOReal(String cameraName) {
        this.name = cameraName;
    }

    // *Called by Vision each loop to seed LL orientation.
    public void setRobotYaw(double yawDegrees) {
        LimelightHelpers.SetRobotOrientation(
            name, yawDegrees, 0, 0, 0, 0, 0
        );
    }

    // *Update IO
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.name = name;
        inputs.hasTarget = LimelightHelpers.getTV(name);

        if (!inputs.hasTarget) {
            clear(inputs);
            return;
        }

        LimelightResults results = LimelightHelpers.getLatestResults(name);
        LimelightTarget_Fiducial[] fiducials = results.targets_Fiducials;

        int tagCount = fiducials.length;

        if (tagCount < 1) {
            clear(inputs);
            return;
        }

        int[] ids = new int[tagCount];
        Pose2d[] poses = new Pose2d[tagCount];

        for (int i = 0; i < tagCount; i++) {
            int id = (int) fiducials[i].fiducialID;
            ids[i] = id;

            var tagFieldPose = VisionConstants.aprilTagLayoutAndymark.getTagPose(id);
            poses[i] = tagFieldPose.isPresent()
                ? tagFieldPose.get().toPose2d()
                : new Pose2d();
        }

        inputs.visibleTagIds   = ids;
        inputs.visibleTagPoses = poses;

        PoseEstimate pe = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        if (!LimelightHelpers.validPoseEstimate(pe)) {
            clearPose(inputs);
            return;
        }

        Pose2d pose = pe.pose;

        if (pe.tagCount < 1) {
            clearPose(inputs);
            return;
        }

        if (pose.getX() < 0 || pose.getX() > Constants.FIELD_MAX_X ||
            pose.getY() < 0 || pose.getY() > Constants.FIELD_MAX_Y) {
            clearPose(inputs);
            return;
        }

        double age = Timer.getFPGATimestamp() - pe.timestampSeconds;
        if (age > 0.25) {
            clearPose(inputs);
            return;
        }

        if (lastGoodPose != null) {
            double jump = pose.getTranslation().getDistance(lastGoodPose.getTranslation());
            if (jump > 2.0) {
                clearPose(inputs);
                return;
            }
        }

        // |Save last good pose
        lastGoodPose = pose;

        inputs.hasEstimatedPose       = true;
        inputs.estimatedPose          = pose;
        inputs.estimatedPoseTimestamp = Utils.fpgaToCurrentTime(pe.timestampSeconds);
        inputs.numTagsUsed            = pe.tagCount;
        inputs.stdDevs = calculateStdDevs(pe);

        inputs.targetId = ids[0]; // best target = first fiducial
    }

        private Matrix<N3, N1> calculateStdDevs(PoseEstimate pe) {
            if (pe.tagCount == 0) {
                return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            }

            double avgDist = pe.avgTagDist;
            int tagCount = pe.tagCount;

            // Base noise floor (0.05m) + distance squared penalty divided by tag count
            double xyStdDev = 0.05 + (0.08 * Math.pow(avgDist, 2) / tagCount);

            // If MegaTag2 / gyro-assisted vision is used, trust translation down to ~0.03m
            xyStdDev = Math.max(xyStdDev, 0.03);

            // Trust gyro completely for theta by setting rotation std dev to infinity
            return VecBuilder.fill(xyStdDev, xyStdDev, Double.MAX_VALUE);
        }

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
}
