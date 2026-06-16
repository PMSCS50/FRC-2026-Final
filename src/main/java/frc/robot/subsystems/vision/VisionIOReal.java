package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * Real hardware implementation of VisionIO using a Limelight 4.
 *
 * Mirrors VisionSubsystem (the non-AK Limelight subsystem) exactly —
 * all the same LimelightHelpers calls, same std-dev logic, same
 * tagToRobot and per-tag HashMap semantics — but writes results into
 * VisionIOInputs so AdvantageKit can log and replay everything.
 *
 * SetRobotOrientation() is called here (not in the subsystem) so that
 * MegaTag2 is always seeded before the pose estimate is read, even in
 * the IO layer.
 */
public class VisionIOReal implements VisionIO {

    private final String llName;

    // *Camera mounting — must match VisionSubsystem constants
    private static final double CAM_FORWARD_M =  0.072;
    private static final double CAM_SIDE_M    = -0.072;
    private static final double CAM_UP_M      =  0.495;
    private static final double CAM_ROLL_DEG  =  0.0;
    private static final double CAM_PITCH_DEG = -Math.toDegrees(Math.toRadians(10)); // store as degrees for LL
    private static final double CAM_YAW_DEG   =  0.0;

    /**
     * @param cameraName  Limelight NT hostname (e.g. "limelight" or "limelight-front")
     * @param robotYawDegSupplier  Supplier for current robot yaw — used to seed MegaTag2.
     *                             Pass drivetrain.getState().Pose.getRotation().getDegrees()
     *                             from your periodic call.
     */
    public VisionIOReal(String cameraName) {
        this.llName = cameraName;

        // *Tell the Limelight where the camera is mounted once at construction
        LimelightHelpers.setCameraPose_RobotSpace(
            llName,
            CAM_FORWARD_M, CAM_SIDE_M, CAM_UP_M,
            CAM_ROLL_DEG,  CAM_PITCH_DEG, CAM_YAW_DEG
        );
    }

    /**
     * Called every loop by VisionSimSystem.periodic().
     * Passes the current robot yaw so MegaTag2 can be seeded before reading.
     */
    public void setRobotYaw(double yawDegrees) {
        LimelightHelpers.SetRobotOrientation(
            llName, yawDegrees, 0, 0, 0, 0, 0
        );
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {

        // *--- Primary target presence & ID ------------------------------------
        inputs.hasTarget = LimelightHelpers.getTV(llName);

        if (!inputs.hasTarget) {
            inputs.targetId        = -1;
            inputs.hasTagTransform = false;
            inputs.tagToRobotX     = 0.0;
            inputs.tagToRobotY     = 0.0;
            inputs.tagToRobotZ     = 0.0;
            inputs.tagToRobotRotZ  = 0.0;
            inputs.visibleTagIds       = new int[0];
            inputs.allTagToRobotX      = new double[0];
            inputs.allTagToRobotY      = new double[0];
            inputs.allTagToRobotZ      = new double[0];
            inputs.allTagToRobotRotZ   = new double[0];
            inputs.hasEstimatedPose    = false;
            inputs.distanceToHub       = 0.0;
            inputs.visionStdDevs = new double[] {
                Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
            };
            return;
        }

        inputs.targetId = (int) LimelightHelpers.getFiducialID(llName);

        // *--- Primary tagToRobot (botpose_targetspace) -------------------------
        // ?botpose_targetspace = robot's pose in the primary tag's frame = tagToRobot
        Pose3d primaryTagToRobot = LimelightHelpers.getBotPose3d_TargetSpace(llName);
        inputs.hasTagTransform = true;
        inputs.tagToRobotX    = primaryTagToRobot.getX();
        inputs.tagToRobotY    = primaryTagToRobot.getY();
        inputs.tagToRobotZ    = primaryTagToRobot.getZ();
        inputs.tagToRobotRotZ = primaryTagToRobot.getRotation().getZ();

        // *--- Per-tag HashMap → parallel arrays --------------------------------
        // ?getLatestResults() parses full JSON; each fiducial exposes 
        // ?getRobotPose_TargetSpace() = tagToRobot for that specific tag.
        LimelightResults results = LimelightHelpers.getLatestResults(llName);
        LimelightTarget_Fiducial[] fiducials = results.targets_Fiducials;

        int n = fiducials.length;
        int[]    ids   = new int[n];
        double[] txArr = new double[n];
        double[] tyArr = new double[n];
        double[] tzArr = new double[n];
        double[] trArr = new double[n];

        for (int i = 0; i < n; i++) {
            Pose3d t2r = fiducials[i].getRobotPose_TargetSpace();
            ids[i]   = (int) fiducials[i].fiducialID;
            txArr[i] = t2r.getX();
            tyArr[i] = t2r.getY();
            tzArr[i] = t2r.getZ();
            trArr[i] = t2r.getRotation().getZ();
        }

        inputs.visibleTagIds     = ids;
        inputs.allTagToRobotX    = txArr;
        inputs.allTagToRobotY    = tyArr;
        inputs.allTagToRobotZ    = tzArr;
        inputs.allTagToRobotRotZ = trArr;

        // *--- Hub distance (tag-space geometry) --------------------------------
        //inputs.distanceToHub = computeHubDistance(inputs);

        // *--- MegaTag2 pose estimate -------------------------------------------
        PoseEstimate pe = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);

        if (!LimelightHelpers.validPoseEstimate(pe)) {
            inputs.hasEstimatedPose = false;
            inputs.visionStdDevs = new double[] {
                Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
            };
            return;
        }

        inputs.hasEstimatedPose       = true;
        inputs.estimatedPose          = pe.pose;
        inputs.estimatedPoseTimestamp = pe.timestampSeconds;
        inputs.numTagsUsed            = pe.tagCount;
        inputs.avgTagDistMeters       = pe.avgTagDist;

        // *MegaTag2 std-devs
        if (pe.tagCount >= 2) {
            inputs.visionStdDevs = new double[] {
                0.5 * pe.avgTagDist,
                0.5 * pe.avgTagDist,
                Math.toRadians(5) // 5 degrees in radians
            };
        } else {
            inputs.visionStdDevs = new double[] {
                1.0 * pe.avgTagDist,
                1.0 * pe.avgTagDist,
                Math.toRadians(10) // 10 degrees in radians
            };
        }
    }
}