package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.Constants.VisionConstants;

/**
 * Real hardware implementation of VisionIO.
 * Talks directly to a physical PhotonCamera over NetworkTables.
 */
public class VisionIOReal implements VisionIO {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;

    // Robot-to-camera transform (same as original)
    private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(0.25, -0.072, 0.09),
        new Rotation3d(0, Math.toRadians(10), 0)
    );

    public VisionIOReal(String cameraName) {
        this.camera = new PhotonCamera(cameraName);

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

    @Override
    public void updateInputs(VisionIOInputs inputs) {
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

            // Compute std devs
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
}
