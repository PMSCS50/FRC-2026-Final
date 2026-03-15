package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.Constants.VisionConstants;

/**
 * AdvantageKit simulation implementation of VisionIO.
 *
 * Uses PhotonVision's VisionSystemSim driven by the robot's simulated
 * ground-truth pose (supplied each cycle by the subsystem/drivetrain).
 *
 * Follows the AK pattern: all real sensor calls happen here, results are
 * written into VisionIOInputs, and the subsystem only reads inputs.
 */
public class VisionIOSim implements VisionIO {

    // Robot-to-camera transform (matches VisionIOReal)
    private static final Transform3d ROBOT_TO_CAMERA = VisionConstants.robotToCamera1;

    private final VisionSystemSim visionSim;
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final PhotonPoseEstimator photonPoseEstimator;

    /**
     * @param cameraName Name of the simulated camera (must match NT name)
     */
    public VisionIOSim(String cameraName) {
        // VisionSystemSim owns the simulated field + all camera sims
        visionSim = new VisionSystemSim("main");

        if (VisionConstants.aprilTagLayout != null) {
            visionSim.addAprilTags(VisionConstants.aprilTagLayout);
        }

        // Camera intrinsics — adjust as needed to match your real camera
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(960, 720, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90));
        cameraProperties.setCalibError(0.25, 0.08);
        cameraProperties.setFPS(20);
        cameraProperties.setAvgLatencyMs(35);
        cameraProperties.setLatencyStdDevMs(5);

        camera = new PhotonCamera(cameraName);
        cameraSim = new PhotonCameraSim(camera, cameraProperties);

        // Enable wireframe rendering in the sim dashboard (optional, useful for tuning)
        cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraSim, ROBOT_TO_CAMERA);

        if (VisionConstants.aprilTagLayout != null) {
            photonPoseEstimator = new PhotonPoseEstimator(
                VisionConstants.aprilTagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                ROBOT_TO_CAMERA
            );
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } else {
            photonPoseEstimator = null;
        }
    }

    /**
     * Must be called each loop with the robot's true simulated pose so that
     * VisionSystemSim can ray-cast against the field and produce synthetic detections.
     *
     * In your subsystem's periodic(), pass drivetrain.getSimulatedDriveTrainPose()
     * (or equivalent) here.
     */
    public void updateSimPose(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Read latest synthetic result from sim camera
        PhotonPipelineResult result = camera.getLatestResult();

        // --- Tag-relative tracking ---
        if (result.hasTargets()) {
            PhotonTrackedTarget best = result.getBestTarget();
            inputs.hasTarget = true;
            inputs.targetId = best.getFiducialId();

            Transform3d cameraToTag = best.getBestCameraToTarget();
            Transform3d robotToTag = ROBOT_TO_CAMERA.plus(cameraToTag);
            Transform3d tagToRobot = robotToTag.inverse();

            inputs.hasTagTransform = true;
            inputs.tagToRobotX = tagToRobot.getX();
            inputs.tagToRobotY = tagToRobot.getY();
            inputs.tagToRobotZ = tagToRobot.getZ();
            inputs.tagToRobotRotZ = tagToRobot.getRotation().getZ();

            // Update inputs.visibleTagIds
            inputs.visibleTagIds = result.getTargets().stream()
                .mapToInt(PhotonTrackedTarget::getFiducialId)
                .toArray();
        } else {
            inputs.hasTarget = false;
            inputs.targetId = -1;
            inputs.hasTagTransform = false;
            inputs.tagToRobotX = 0.0;
            inputs.tagToRobotY = 0.0;
            inputs.tagToRobotZ = 0.0;
            inputs.tagToRobotRotZ = 0.0;
        }

        // --- Pose estimation ---
        if (photonPoseEstimator == null) {
            inputs.hasEstimatedPose = false;
            return;
        }

        Optional<EstimatedRobotPose> latestEst = Optional.empty();
        List<PhotonTrackedTarget> latestTargets = List.of();

        for (PhotonPipelineResult r : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> est = photonPoseEstimator.update(r);
            if (est.isPresent()) {
                latestEst = est;
                latestTargets = r.getTargets();
            }
        }

        if (latestEst.isPresent()) {
            EstimatedRobotPose est = latestEst.get();
            inputs.hasEstimatedPose = true;
            inputs.estimatedPose = est.estimatedPose.toPose2d();
            inputs.estimatedPoseTimestamp = est.timestampSeconds;

            int numTags = latestTargets.size();
            double avgDist = 0.0;
            for (PhotonTrackedTarget t : latestTargets) {
                avgDist += t.getBestCameraToTarget().getTranslation().getNorm();
            }
            avgDist = numTags > 0 ? avgDist / numTags : 0.0;

            inputs.numTagsUsed = numTags;
            inputs.avgTagDistMeters = avgDist;

            if (numTags >= 2) {
                inputs.visionStdDevs = new double[] {
                    0.5 * avgDist,
                    0.5 * avgDist,
                    Math.toRadians(5)
                };
            } else {
                inputs.visionStdDevs = new double[] {
                    1.0 * avgDist,
                    1.0 * avgDist,
                    Math.toRadians(10)
                };
            }
        } else {
            inputs.hasEstimatedPose = false;
            inputs.visionStdDevs = new double[] {
                Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
            };
        }
    }

    /** Exposes the VisionSystemSim for dashboard/debug use (e.g., SimGUI field overlay). */
    public VisionSystemSim getVisionSim() {
        return visionSim;
    }
}
