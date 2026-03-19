package frc.robot.subsystems;

import java.util.HashMap;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;
    private final CommandSwerveDrivetrain drivetrain;
    private final PhotonPoseEstimator photonPoseEstimator;

    private static AprilTagFieldLayout tagFieldLayout;
    /*
    private static final Transform3d ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(0.072, -.072, 0.495),
            new Rotation3d(0, Math.toRadians(10), 0)
        );
    */

    //If our camera is tilted 10 degrees up
    //then pitch must be negative
    private static final Transform3d ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(0.072, -.072, 0.495),
            new Rotation3d(0, -Math.toRadians(10), 0)
        );

    private boolean hasTarget = false;
    private int targetId = -1;
    private PhotonPipelineResult result = null;
    private final HashMap<Integer, Transform3d> tagTransforms = new HashMap<>();
    private final HashMap<Integer, PhotonTrackedTarget> tagPhotonTrack = new HashMap<>();

    public VisionSubsystem(String cameraName, CommandSwerveDrivetrain drivetrain) {
        this.camera = new PhotonCamera(cameraName);
        this.drivetrain = drivetrain;

        try {
            tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        } catch (Exception e) {
            tagFieldLayout = null;
            DriverStation.reportError("VisionSubsystem: Failed to load field layout - " + e.getMessage(), false);
        }

        if (tagFieldLayout != null) {
            photonPoseEstimator = new PhotonPoseEstimator(
                tagFieldLayout,
                //PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, [Deprecated. PPE now has builtin methods for each strategy that we use instead]
                ROBOT_TO_CAMERA
            );
            //photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); [Also deprecated]
        } else {
            photonPoseEstimator = null;
        }
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult();
        hasTarget = result.hasTargets();
        tagTransforms.clear();

        if (!hasTarget) {
            targetId = -1;
            return;
        }

        for (PhotonTrackedTarget t : result.getTargets()) {
            Transform3d camToTag = t.getBestCameraToTarget();
            Transform3d robotToTag = ROBOT_TO_CAMERA.plus(camToTag);
            tagTransforms.put(t.getFiducialId(), robotToTag.inverse());
            tagPhotonTrack.put(t.fiducialId, t);
        }

        targetId = result.getBestTarget().getFiducialId();

        fieldToRobot = vision.estimateMultiTagPose();
        fieldToRobot.ifPresent(
            erp -> {
                Matrix<N3, N1> visionStdDevs = vision.getEstimationStdDevs();
                drivetrain.addVisionMeasurement(erp.estimatedPose.toPose2d(), erp.timestampSeconds, visionStdDevs);
            }
        );
    }

    // ************************
    // GETTER METHODS
    // ************************

    public boolean hasTarget(int id) {
        return tagTransforms.containsKey(id);
    }

    public boolean hasTargets() {
        return hasTarget;
    }

    public int getTargetId() {
        return targetId;
    }

    public PhotonPipelineResult getResult() {
        return result;
    }

    public Transform3d getTransformToTag(int id) {
        return tagTransforms.getOrDefault(id, null);
    }

    public double getX(int id) {
        Transform3d tag = tagTransforms.get(id);
        return tag != null ? tag.getX() : 0.0;
    }

    public double getY(int id) {
        Transform3d tag = tagTransforms.get(id);
        return tag != null ? tag.getY() : 0.0;
    }

    public double getYawRad(int id) {
        Transform3d tag = tagTransforms.get(id);
        if (tag == null) return 0.0;
        return MathUtil.angleModulus(tag.getRotation().getZ() - Math.PI);
    }

    public double getTargetYaw(int id) {
        Transform3d tag = tagTransforms.get(id);
        PhotonTrackedTarget t = tagPhotonTrack.get(id);
        if (tag == null) return 0.0;
        double targetYaw = t.getYaw();
        return targetYaw;

    }


    public double getDistance(int id) {
        Transform3d tag = tagTransforms.get(id);
        return tag != null ? Math.hypot(tag.getX(), tag.getY()) : 0.0;
    }

    // public double getDistance2(int id) {
    //     Transform3d tag = tagTransforms.get(id);
    //     return tag != null ? Math.hypot(tag.getX() + 0.6, tag.getY()) : 0.0;
    // }

    public double getYawToPose(Pose2d targetPose) {
        if (drivetrain.getPose() == null) return 0.0;
        return PhotonUtils.getYawToPose(drivetrain.getPose(), targetPose).getRadians();
    }

    public double getDistanceToPose(Pose2d targetPose) {
        if (drivetrain.getPose() == null) return 0.0;
        return PhotonUtils.getDistanceToPose(drivetrain.getPose(), targetPose);
    }

    // ************************
    // SHOOTER HELPERS
    // ************************

    double shooterHeight = 0.508;
    double phi = Math.toRadians(70);

    // public double rpmFromDistance(double distance) {
    //     double y = 1.8288 - shooterHeight;
    //     double shooterVelocity = Math.sqrt(
    //         (9.807 * distance * distance) /
    //         (2 * Math.cos(phi) * Math.cos(phi) * (distance * Math.tan(phi) - y))
    //     );
    //     double dragFactor = (1 + 0.0000001 * distance * distance) * 1.031;
    //     shooterVelocity *= dragFactor;
    //     double wheelRadius = 0.0508;
    //     double c = 1.06;
    //     double rpm = c * (shooterVelocity * 60.0) / (Math.PI * wheelRadius);
    //     SmartDashboard.putNumber("Shooter rpm", rpm);
    //     return rpm;
    // }

    public double rpmFromDistanceRegression(double distance) {
        double rps = -0.3039109023 * distance * distance 
                + 6.81380687 * distance 
                + 40.82402705 - .5;
        SmartDashboard.putNumber("Shooter rps regression", rps);
        double rpm = rps * 60;
        SmartDashboard.putNumber("Shooter rpm regression", rpm);
        return rpm;
    }

    
    //***********************
    //POSE ESTIMATION HELPERS
    //***********************

    private Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0,0,0);

    //Ambiguity of Photon poseEstimation
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose,
    List<PhotonTrackedTarget> targets) {

        int numTags = targets.size();

        // Average distance from camera to visible AprilTags
        double avgDist = 0.0;
        for (PhotonTrackedTarget target : targets) {
            avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        avgDist /= numTags;

        if (numTags >= 2) {
            visionStdDevs = VecBuilder.fill(
                    0.5 * avgDist,
                    0.5 * avgDist,
                    Math.toRadians(5));
        } else {
            visionStdDevs = VecBuilder.fill(
                    1.0 * avgDist,
                    1.0 * avgDist,
                    Math.toRadians(10));
        }
    }
    
    public Matrix<N3, N1> getEstimationStdDevs() {
        return visionStdDevs;
    }

    //gets fieldToRobot
    public Optional<EstimatedRobotPose> estimateMultiTagPose() {

        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var result : camera.getAllUnreadResults()) {
            visionEst = photonPoseEstimator.estimateCoprocMultiTagPose(result);
            if (vision.isEmpty()) {
                visionEst = photonPoseEstimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());
        }
        
        return visionEst;
    }
}
