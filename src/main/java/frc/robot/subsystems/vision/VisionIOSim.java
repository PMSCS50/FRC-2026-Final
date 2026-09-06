package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class VisionIOSim implements VisionIO {

    private final VisionSystemSim visionSim;
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final PhotonPoseEstimator poseEstimator;
    private final String name;

    private Pose2d lastGoodPose = null;

    public VisionIOSim(String cameraName, Transform3d robotToCamera) {
        visionSim = new VisionSystemSim("simVision");
        this.name = cameraName;

        if (VisionConstants.aprilTagLayoutAndymark != null) {
            visionSim.addAprilTags(VisionConstants.aprilTagLayoutAndymark);
        }

        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(960, 720, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90));
        props.setCalibError(0.05, 0.02);
        props.setFPS(30);
        props.setAvgLatencyMs(20);
        props.setLatencyStdDevMs(5);

        camera    = new PhotonCamera(name);
        cameraSim = new PhotonCameraSim(camera, props);
        cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraSim, robotToCamera);

        poseEstimator = new PhotonPoseEstimator(
            VisionConstants.aprilTagLayoutAndymark,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_RIO,
            robotToCamera
        );
    }

    public void updateSimPose(Pose2d robotPose) {
        visionSim.update(robotPose);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.name = name;
        PhotonPipelineResult result = camera.getLatestResult();

        if (result == null) {
            clear(inputs);
            return;
        }

        inputs.hasTarget = result.hasTargets();
        if (!inputs.hasTarget) {
            clear(inputs);
            return;
        }

        List<PhotonTrackedTarget> targets = result.getTargets();
        int tagCount = targets.size();

        // Allow single-tag solves; fusion layer will handle quality
        if (tagCount < 1) {
            clear(inputs);
            return;
        }

        int[] ids = new int[tagCount];
        Pose2d[] poses = new Pose2d[tagCount];

        for (int i = 0; i < tagCount; i++) {
            PhotonTrackedTarget t = targets.get(i);
            ids[i] = t.getFiducialId();

            Optional<Pose3d> tagFieldPose =
                VisionConstants.aprilTagLayoutAndymark.getTagPose(ids[i]);

            poses[i] = tagFieldPose.isPresent()
                ? tagFieldPose.get().toPose2d()
                : new Pose2d();
        }

        inputs.visibleTagIds   = ids;
        inputs.visibleTagPoses = poses;

        Optional<EstimatedRobotPose> est = poseEstimator.update(result);
        if (est.isEmpty()) {
            clearPose(inputs);
            return;
        }

        EstimatedRobotPose erp = est.get();
        Pose2d pose = erp.estimatedPose.toPose2d();

        if (pose.getX() < 0 || pose.getX() > Constants.FIELD_MAX_X ||
            pose.getY() < 0 || pose.getY() > Constants.FIELD_MAX_Y) {
            clearPose(inputs);
            return;
        }

        double age = Timer.getFPGATimestamp() - erp.timestampSeconds;
        if (age > 0.25) {
            clearPose(inputs);
            return;
        }

        // Save last good pose
        lastGoodPose = pose;

        inputs.hasEstimatedPose       = true;
        inputs.estimatedPose          = pose;
        inputs.estimatedPoseTimestamp = erp.timestampSeconds;
        inputs.numTagsUsed            = tagCount;

        PhotonTrackedTarget best = result.getBestTarget();
        inputs.targetId = (best != null) ? best.getFiducialId() : -1;
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

    public VisionSystemSim getVisionSim() {
        return visionSim;
    }
}
