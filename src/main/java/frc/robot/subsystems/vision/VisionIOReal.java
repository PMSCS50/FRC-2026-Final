package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class VisionIOReal implements VisionIO {

    private final String llName;
    private final Transform3d robotToCamera;

    private Pose2d lastGoodPose = null;

    /**
     * Real camera IO with configurable mounting.
     *
     * @param cameraName    Limelight name
     * @param robotToCamera Transform from robot origin to camera
     */
    public VisionIOReal(String cameraName, Transform3d robotToCamera) {
        this.llName = cameraName;
        this.robotToCamera = robotToCamera;

        // Configure Limelight camera pose from Transform3d
        LimelightHelpers.setCameraPose_RobotSpace(
            llName,
            robotToCamera.getX(),
            robotToCamera.getY(),
            robotToCamera.getZ(),
            robotToCamera.getRotation().getX(),  // roll (deg)
            robotToCamera.getRotation().getY(),  // pitch (deg)
            robotToCamera.getRotation().getZ()   // yaw (deg)
        );
    }

    /** Called by Vision each loop to seed LL orientation. */
    public void setRobotYaw(double yawDegrees) {
        LimelightHelpers.SetRobotOrientation(
            llName, yawDegrees, 0, 0, 0, 0, 0
        );
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.hasTarget = LimelightHelpers.getTV(llName);

        if (!inputs.hasTarget) {
            clear(inputs);
            return;
        }

        LimelightResults results = LimelightHelpers.getLatestResults(llName);
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

        PoseEstimate pe = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);

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

        lastGoodPose = pose;

        inputs.hasEstimatedPose       = true;
        inputs.estimatedPose          = pose;
        inputs.estimatedPoseTimestamp = pe.timestampSeconds;
        inputs.numTagsUsed            = pe.tagCount;

        inputs.targetId = ids[0]; // best target = first fiducial
    }

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
