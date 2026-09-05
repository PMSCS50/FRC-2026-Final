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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

import frc.robot.Constants.VisionConstants;

public class VisionIOSim implements VisionIO {

    private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(0.072, -0.072, 0.495),
        new Rotation3d(0, Math.toRadians(10), 0)
    );

    private final VisionSystemSim visionSim;
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final PhotonPoseEstimator poseEstimator;

    public VisionIOSim(String cameraName) {

        visionSim = new VisionSystemSim("simVision");

        if (VisionConstants.aprilTagLayoutAndymark != null) {
            visionSim.addAprilTags(VisionConstants.aprilTagLayoutAndymark);
        }

        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(960, 720, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90));
        props.setCalibError(0.25, 0.08);
        props.setFPS(30);
        props.setAvgLatencyMs(20);
        props.setLatencyStdDevMs(5);

        camera    = new PhotonCamera(cameraName);
        cameraSim = new PhotonCameraSim(camera, props);
        cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraSim, ROBOT_TO_CAMERA);

        poseEstimator = new PhotonPoseEstimator(
            VisionConstants.aprilTagLayoutAndymark,
            ROBOT_TO_CAMERA
        );
    }

    /** Called by LLSubsystemMany each loop. */
    public void updateSimPose(Pose2d robotPose) {
        visionSim.update(robotPose);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            inputs.hasTarget        = false;
            inputs.targetId         = -1;
            inputs.visibleTagIds    = new int[0];
            inputs.visibleTagPoses  = new Pose2d[0];
            inputs.hasEstimatedPose = false;
            return;
        }

        PhotonPipelineResult result = results.get(results.size() - 1);

        inputs.hasTarget = result.hasTargets();

        if (!inputs.hasTarget) {
            inputs.targetId         = -1;
            inputs.visibleTagIds    = new int[0];
            inputs.visibleTagPoses  = new Pose2d[0];
            inputs.hasEstimatedPose = false;
            return;
        }

        PhotonTrackedTarget best = result.getBestTarget();
        inputs.targetId = best.getFiducialId();

        // Per-tag field-space poses
        List<PhotonTrackedTarget> allTargets = result.getTargets();
        int n = allTargets.size();

        int[] ids = new int[n];
        Pose2d[] poses = new Pose2d[n];

        for (int i = 0; i < n; i++) {
            PhotonTrackedTarget t = allTargets.get(i);
            ids[i] = t.getFiducialId();

            Optional<Pose3d> tagFieldPose = VisionConstants.aprilTagLayoutAndymark.getTagPose(ids[i]);
            poses[i] = tagFieldPose.isPresent()
                ? tagFieldPose.get().toPose2d()
                : new Pose2d();
        }

        inputs.visibleTagIds   = ids;
        inputs.visibleTagPoses = poses;

        // Pose estimate
        Optional<EstimatedRobotPose> est = Optional.empty();
        for (var resultVar : results) {
            est = poseEstimator.estimateCoprocMultiTagPose(resultVar);
            if (est.isEmpty()) {
                est = poseEstimator.estimateLowestAmbiguityPose(resultVar);
            }
        }
        if (est.isEmpty()) {
            inputs.hasEstimatedPose = false;
            return;
        }

        EstimatedRobotPose erp = est.get();

        inputs.hasEstimatedPose       = true;
        inputs.estimatedPose          = erp.estimatedPose.toPose2d();
        inputs.estimatedPoseTimestamp = erp.timestampSeconds;
        inputs.numTagsUsed            = result.getTargets().size();
    }

    public VisionSystemSim getVisionSim() {
        return visionSim;
    }
}
