package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class VisionIOReal implements VisionIO {

    private final String llName;

    // Camera mounting (must match your real robot)
    private static final double CAM_FORWARD_M =  0.072;
    private static final double CAM_SIDE_M    = -0.072;
    private static final double CAM_UP_M      =  0.495;
    private static final double CAM_ROLL_DEG  =  0.0;
    private static final double CAM_PITCH_DEG = -10.0;
    private static final double CAM_YAW_DEG   =  0.0;

    public VisionIOReal(String cameraName) {
        this.llName = cameraName;

        LimelightHelpers.setCameraPose_RobotSpace(
            llName,
            CAM_FORWARD_M, CAM_SIDE_M, CAM_UP_M,
            CAM_ROLL_DEG, CAM_PITCH_DEG, CAM_YAW_DEG
        );
    }

    /** Called by LLSubsystemMany each loop. */
    public void setRobotYaw(double yawDegrees) {
        LimelightHelpers.SetRobotOrientation(
            llName, yawDegrees, 0, 0, 0, 0, 0
        );
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {

        inputs.hasTarget = LimelightHelpers.getTV(llName);

        if (!inputs.hasTarget) {
            inputs.targetId        = -1;
            inputs.visibleTagIds   = new int[0];
            inputs.visibleTagPoses = new Pose2d[0];
            inputs.hasEstimatedPose = false;
            return;
        }

        inputs.targetId = (int) LimelightHelpers.getFiducialID(llName);

        // Get fiducials
        LimelightResults results = LimelightHelpers.getLatestResults(llName);
        LimelightTarget_Fiducial[] fiducials = results.targets_Fiducials;

        int n = fiducials.length;
        int[] ids = new int[n];
        Pose2d[] poses = new Pose2d[n];

        for (int i = 0; i < n; i++) {
            int id = (int) fiducials[i].fiducialID;
            ids[i] = id;

            // Field-space tag pose from AprilTag layout
            var tagFieldPose = VisionConstants.aprilTagLayoutAndymark.getTagPose(id);
            poses[i] = tagFieldPose.isPresent()
                ? tagFieldPose.get().toPose2d()
                : new Pose2d();
        }

        inputs.visibleTagIds   = ids;
        inputs.visibleTagPoses = poses;

        // MegaTag2 robot pose estimate
        PoseEstimate pe = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);

        if (!LimelightHelpers.validPoseEstimate(pe)) {
            inputs.hasEstimatedPose = false;
            return;
        }

        inputs.hasEstimatedPose       = true;
        inputs.estimatedPose          = pe.pose;
        inputs.estimatedPoseTimestamp = pe.timestampSeconds;
        inputs.numTagsUsed            = pe.tagCount;
    }

}
