package frc.robot.subsystems;

import java.util.HashMap;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;
    private final CommandSwerveDrivetrain drivetrain;
    private final PhotonPoseEstimator photonPoseEstimator;

    private static AprilTagFieldLayout tagFieldLayout;

    private static final Transform3d ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(0.072, -.072, 0.495),
            new Rotation3d(0, Math.toRadians(10), 0)
        );

    private boolean hasTarget = false;
    private int targetId = -1;
    private PhotonPipelineResult result = null;
    private final HashMap<Integer, Transform3d> tagTransforms = new HashMap<>();

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
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                ROBOT_TO_CAMERA
            );
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
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
        }

        targetId = result.getBestTarget().getFiducialId();
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
}
