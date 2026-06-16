package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

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
 * // !AdvantageKit simulation implementation of VisionIO.
 * 
 * // ?We can't run LimelightHelpers in simulation (it needs real NT data from
 * // ?the LL firmware), so we keep PhotonVision's VisionSystemSim here to
 * // ?generate synthetic detections — but we write the results into VisionIOInputs
 * // ?using the SAME field layout and std-dev logic as VisionIOReal, so the
 * // ?subsystem layer (VisionSimSystem) sees identical data in sim and on hardware.

 * // *The only sim-specific call is updateSimPose(), which VisionSimSystem calls
 * // *every loop to push the drivetrain's ground-truth pose into VisionSystemSim.
*/

public class VisionIOSim implements VisionIO {

    // *Must match VisionIOReal mounting constants
    private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(0.072, -0.072, 0.495),
        new Rotation3d(0, Math.toRadians(10), 0)
    );

    private final VisionSystemSim visionSim;
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final PhotonPoseEstimator photonPoseEstimator;

    public VisionIOSim(String cameraName) {
        visionSim = new VisionSystemSim("main");

        if (VisionConstants.aprilTagLayoutAndymark != null) {
            visionSim.addAprilTags(VisionConstants.aprilTagLayoutAndymark);
        }

        // *Camera intrinsics — tune to match your real Limelight 4 FOV
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(960, 720, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90));
        cameraProperties.setCalibError(0.25, 0.08);
        cameraProperties.setFPS(30);
        cameraProperties.setAvgLatencyMs(20);
        cameraProperties.setLatencyStdDevMs(5);

        camera    = new PhotonCamera(cameraName);
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraSim, ROBOT_TO_CAMERA);

        if (VisionConstants.aprilTagLayoutAndymark != null) {
            photonPoseEstimator = new PhotonPoseEstimator(VisionConstants.aprilTagLayoutAndymark, ROBOT_TO_CAMERA);
        } else {
            photonPoseEstimator = null;
        }
    }

    // *Push the robot's true simulated pose into VisionSystemSim so it can
    // *ray-cast against the field and produce synthetic tag detections.
    // ?Called every loop by VisionSimSystem.periodic() when in simulation.
    public void updateSimPose(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        
        Optional<EstimatedRobotPose> latestEst = Optional.empty();
        List<PhotonTrackedTarget> latestTargets = List.of();

        if (results.isEmpty()) {
            inputs.hasTarget       = false;
            inputs.targetId        = -1;
            inputs.hasTagTransform = false;
            inputs.tagToRobotX     = 0.0;
            inputs.tagToRobotY     = 0.0;
            inputs.tagToRobotZ     = 0.0;
            inputs.tagToRobotRotZ  = 0.0;
            inputs.visibleTagIds     = new int[0];
            inputs.allTagToRobotX    = new double[0];
            inputs.allTagToRobotY    = new double[0];
            inputs.allTagToRobotZ    = new double[0];
            inputs.allTagToRobotRotZ = new double[0];
            inputs.distanceToHub     = 0.0;
        } else {
            PhotonPipelineResult result = results.get(results.size() - 1); // single declaration

            // *--- Primary target + per-tag arrays
            if (result.hasTargets()) {
                PhotonTrackedTarget best = result.getBestTarget();
                inputs.hasTarget = true;
                inputs.targetId  = best.getFiducialId();

                // *Primary tagToRobot (mirrors VisionIOReal: botpose_targetspace)
                Transform3d cameraToTag = best.getBestCameraToTarget();
                Transform3d robotToTag  = ROBOT_TO_CAMERA.plus(cameraToTag);
                Transform3d tagToRobot  = robotToTag.inverse();

                inputs.hasTagTransform = true;
                inputs.tagToRobotX    = tagToRobot.getX();
                inputs.tagToRobotY    = tagToRobot.getY();
                inputs.tagToRobotZ    = tagToRobot.getZ();
                inputs.tagToRobotRotZ = tagToRobot.getRotation().getZ();

                // *Per-tag parallel arrays (mirrors VisionIOReal's JSON loop)
                List<PhotonTrackedTarget> allTargets = result.getTargets();
                int n = allTargets.size();

                int[]    ids   = new int[n];
                double[] txArr = new double[n];
                double[] tyArr = new double[n];
                double[] tzArr = new double[n];
                double[] trArr = new double[n];

                for (int i = 0; i < n; i++) {
                    PhotonTrackedTarget t    = allTargets.get(i);
                    Transform3d c2t         = t.getBestCameraToTarget();
                    Transform3d r2t         = ROBOT_TO_CAMERA.plus(c2t);
                    Transform3d t2r         = r2t.inverse();
                    ids[i]   = t.getFiducialId();
                    txArr[i] = t2r.getX();
                    tyArr[i] = t2r.getY();
                    tzArr[i] = t2r.getZ();
                    trArr[i] = t2r.getRotation().getZ();
                }

                inputs.visibleTagIds     = ids;
    
                if (VisionConstants.aprilTagLayoutAndymark != null) {
                    List<Pose2d> tagPoses = new ArrayList<>();
                    for (int id : inputs.visibleTagIds) {
                        VisionConstants.aprilTagLayoutAndymark
                            .getTagPose(id)
                            .ifPresent(pose -> tagPoses.add(pose.toPose2d()));
                    }
                    inputs.visibleTagPoses = tagPoses.toArray(new Pose2d[0]);
                }

                inputs.allTagToRobotX    = txArr;
                inputs.allTagToRobotY    = tyArr;
                inputs.allTagToRobotZ    = tzArr;
                inputs.allTagToRobotRotZ = trArr;
            } else {
                inputs.hasTarget       = false;
                inputs.targetId        = -1;
                inputs.hasTagTransform = false;
                inputs.tagToRobotX     = 0.0;
                inputs.tagToRobotY     = 0.0;
                inputs.tagToRobotZ     = 0.0;
                inputs.tagToRobotRotZ  = 0.0;
                inputs.visibleTagIds     = new int[0];
                inputs.allTagToRobotX    = new double[0];
                inputs.allTagToRobotY    = new double[0];
                inputs.allTagToRobotZ    = new double[0];
                inputs.allTagToRobotRotZ = new double[0];
                inputs.distanceToHub     = 0.0;
            }

            // *--- Pose estimation
            if (photonPoseEstimator != null) {
                latestEst = photonPoseEstimator.estimateCoprocMultiTagPose(result);
                if (latestEst.isEmpty()) {
                    latestEst = photonPoseEstimator.estimateLowestAmbiguityPose(result);
                }
                latestTargets = latestEst.isPresent() ? result.getTargets() : List.of();
            }
        }

        // *--- Pose estimation --------------------------------------------------
        if (photonPoseEstimator == null) {
            inputs.hasEstimatedPose = false;
            inputs.visionStdDevs = new double[] {
                Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
            };
            return;
        }

        if (latestEst.isPresent()) {
            EstimatedRobotPose est = latestEst.get();
            inputs.hasEstimatedPose       = true;
            inputs.estimatedPose          = est.estimatedPose.toPose2d();
            inputs.distanceToHub          = inputs.estimatedPose.getTranslation().getDistance(VisionConstants.getHubPose().getTranslation());
            inputs.estimatedPoseTimestamp = est.timestampSeconds;

            int    numTags = latestTargets.size();
            double avgDist = 0.0;
            for (PhotonTrackedTarget t : latestTargets) {
                avgDist += t.getBestCameraToTarget().getTranslation().getNorm();
            }
            avgDist = numTags > 0 ? avgDist / numTags : 0.0;

            inputs.numTagsUsed      = numTags;
            inputs.avgTagDistMeters = avgDist;

            // *Match VisionIOReal std-dev logic exactly (MegaTag2 style: pin yaw)
            if (numTags >= 2) {
                inputs.visionStdDevs = new double[] {
                    0.5 * avgDist,
                    0.5 * avgDist,
                    9999.0
                };
            } else {
                inputs.visionStdDevs = new double[] {
                    1.0 * avgDist,
                    1.0 * avgDist,
                    9999.0
                };
            }
        } else {
            inputs.hasEstimatedPose = false;
            inputs.visionStdDevs = new double[] {
                Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
            };
        }
    }

    /** Exposes VisionSystemSim for SimGUI field overlay / debug use. */
    public VisionSystemSim getVisionSim() {
        return visionSim;
    }
}