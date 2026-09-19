package frc.robot.subsystems.vision2;

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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class VisionIOSim implements VisionIO {

    private final VisionSystemSim visionSim;
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final PhotonPoseEstimator poseEstimator;
    private final String name;
    private final Transform3d robotToCamera;

    private Pose2d lastGoodPose = null;

    /**
     * Simulated camera IO with configurable mounting.
     *
     * @param cameraName    Limelight name
     * @param robotToCamera Transform from robot origin to camera
     */
    public VisionIOSim(String cameraName, Transform3d robotToCamera) {
        visionSim = new VisionSystemSim("simVision");
        this.name = cameraName;
        this.robotToCamera = robotToCamera;

        if (VisionConstants.aprilTagLayoutAndymark != null) {
            visionSim.addAprilTags(VisionConstants.aprilTagLayoutAndymark);
        }

        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(960, 720, Rotation2d.fromDegrees(90));
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
            robotToCamera
        );
    }

    // *Called by Vision each loop to seed PV orientation.
    public void updateSimPose(Pose2d robotPose) {
        poseEstimator.addHeadingData(Timer.getTimestamp(), robotPose.getRotation());
        visionSim.update(robotPose);
    }

    // *Update IO
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        //Latest Result
        if (results.isEmpty()) {
            clear(inputs);
            return;
        }
        
        PhotonPipelineResult result = results.get(results.size() - 1);

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

            Transform3d tagToRobotTransform = t.getBestCameraToTarget().plus(robotToCamera.inverse());

            poses[i] = new Pose2d(
                tagToRobotTransform.getX(),
                tagToRobotTransform.getY(),
                tagToRobotTransform.getRotation().toRotation2d()
            );
        }

        inputs.visibleTagIds   = ids;
        inputs.visibleTagPoses = poses;

        Optional<EstimatedRobotPose> est = Optional.empty();

        for (PhotonPipelineResult r : results) {
            est = poseEstimator.estimateCoprocMultiTagPose(r);
            if (est.isEmpty()) {
                est = poseEstimator.estimateLowestAmbiguityPose(r);
            }
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

        // |Save last good pose
        lastGoodPose = pose;

        inputs.hasEstimatedPose       = true;
        inputs.estimatedPose          = pose;
        inputs.estimatedPoseTimestamp = erp.timestampSeconds;
        inputs.numTagsUsed            = tagCount;

        inputs.stdDevs = calculateStdDevs(erp);

        PhotonTrackedTarget best = result.getBestTarget();
        inputs.targetId = (best != null) ? best.getFiducialId() : -1;
    }

    private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose erp) {
        if (erp.targetsUsed.isEmpty()) {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }

        double avgDist = 0; 
        for (PhotonTrackedTarget target : erp.targetsUsed) {
            avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        int tagCount = erp.targetsUsed.size();
        avgDist /= tagCount;

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

    // *Getters
    public VisionSystemSim getVisionSim() {
        return visionSim;
    }
}
