package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.VisionConstants.*;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;
    private final CommandSwerveDrivetrain drivetrain;
    private final PhotonPoseEstimator photonPoseEstimator;

    // Field layout initialization
    private static AprilTagFieldLayout tagFieldLayout;

    // Robot to Camera Transform
    private static final Transform3d ROBOT_TO_CAMERA =
        // new Transform3d(
        //     new Translation3d(0.072, -.072, 0.495),
        //     new Rotation3d(0, Math.toRadians(10), 0)
        // );
        new Transform3d(
            new Translation3d(0.25, -.072, 0.09),
            new Rotation3d(0, Math.toRadians(10), 0)
        );

    // Raw vision state
    private PhotonTrackedTarget target;
    private boolean hasTarget = false;
    private int targetId = -1;
    private Transform3d tagToRobot = null;

    // Pose estimation standard deviations
    // Lower values = trust vision more. Higher values = trust vision less.
    private Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0, 0, 0);




    public VisionSubsystem(String cameraName, CommandSwerveDrivetrain drivetrain) {
        
        this.camera = new PhotonCamera(cameraName);
        this.drivetrain = drivetrain;

        // Load field layout with error handling
        // If this fails, vision pose estimation will be disabled but
        // tag-relative alignment (getX, getY, getYawRad) still works
        try {
            tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        } catch (Exception e) {
            tagFieldLayout = null;
            DriverStation.reportError("VisionSubsystem: Failed to load field layout - " + e.getMessage(), false);
        }

        // Set up pose estimator
        // MULTI_TAG_PNP_ON_COPROCESSOR uses multiple tags for a more accurate estimate
        // Falls back to LOWEST_AMBIGUITY when only one tag is visible
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

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    @Override
    public void periodic() {
        // --------------------
        // Part 1: Tag-relative tracking (getX, getY, getYawRad)
        // This is the same as before — used by PV_Align and FaceTag
        // --------------------
        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) {
            hasTarget = false;
            targetId = -1;
            tagToRobot = null;
        } else {
            target = result.getBestTarget();
            hasTarget = true;
            targetId = target.getFiducialId();

            // Camera - Tag
            Transform3d cameraToTag = target.getBestCameraToTarget();

            // Robot - Tag
            Transform3d robotToTag = ROBOT_TO_CAMERA.plus(cameraToTag);

            // Tag - Robot
            tagToRobot = robotToTag.inverse();
        }

        // Part 2: Field-relative pose estimation
        // Feeds vision measurements into drivetrain pose estimator
        // Only runs if field layout loaded successfully
        if (photonPoseEstimator == null) return;

        Optional<EstimatedRobotPose> estimatedPose = estimateMultiTagPose();

        estimatedPose.ifPresent(est -> {
            // Update standard deviations based on how many tags we see and how far away they are
            updateEstimationStdDevs(estimatedPose, est.targetsUsed);

            // Feed the vision measurement into the drivetrain's pose estimator
            // This improves odometry accuracy over time
            drivetrain.addVisionMeasurement(
                est.estimatedPose.toPose2d(),
                est.timestampSeconds,
                visionStdDevs
            );
        });
    }


    /**
     * Attempts to estimate the robot's field-relative pose using visible AprilTags.
     * Uses multi-tag PNP when multiple tags are visible, falls back to lowest
     * ambiguity single-tag when only one tag is visible.
     */
    private Optional<EstimatedRobotPose> estimateMultiTagPose() {
        Optional<EstimatedRobotPose> latestEstimate = Optional.empty();

        // getAllUnreadResults() gets every frame since the last call
        // We process all of them and keep the most recent valid estimate
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> est = photonPoseEstimator.update(result);
            if (est.isPresent()) {
                latestEstimate = est;
                updateEstimationStdDevs(est, result.getTargets());
            }
        }

        return latestEstimate;
    }

    /**
     * Updates visionStdDevs based on number of visible tags and their distance.
     *
     * With 2+ tags: lower standard deviations (trust vision more)
     * With 1 tag:  higher standard deviations (trust vision less)
     * Further away: higher standard deviations (measurements less reliable at range)
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose,
            List<PhotonTrackedTarget> targets) {

        if (estimatedPose.isEmpty() || targets.isEmpty()) {
            // No estimate — use very high std devs so drivetrain ignores this measurement
            visionStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            return;
        }

        int numTags = targets.size();

        // Calculate average distance from camera to all visible tags
        double avgDist = 0.0;
        for (PhotonTrackedTarget tgt : targets) {
            avgDist += tgt.getBestCameraToTarget().getTranslation().getNorm();
        }
        avgDist /= numTags;

        if (numTags >= 2) {
            // Multiple tags visible — high confidence
            // Scale std devs with distance (further = less confident)
            visionStdDevs = VecBuilder.fill(
                0.5 * avgDist,   // X std dev (meters)
                0.5 * avgDist,   // Y std dev (meters)
                Math.toRadians(5) // Rotation std dev (radians) — kept tighter
            );
        } else {
            // Single tag — lower confidence, scale more aggressively with distance
            visionStdDevs = VecBuilder.fill(
                1.0 * avgDist,    // X std dev (meters)
                1.0 * avgDist,    // Y std dev (meters)
                Math.toRadians(10) // Rotation std dev (radians)
            );
        }
    }


    // ************************
    // GETTER METHODS
    // ************************

    /** Returns true if the camera currently sees the specified tag ID */
    public boolean hasTarget(int desiredId) {
        return hasTarget && targetId == desiredId && tagToRobot != null;
    }

    /** Returns true if the camera currently sees any target */
    public boolean hasTargets() {
        return hasTarget;
    }

    /** Returns the ID of the best visible tag, or -1 if no target */
    public int getTargetId() {
        return targetId;
    }

    /** Forward/back distance from robot to tag face (meters) */
    public double getX() {
        return tagToRobot != null ? tagToRobot.getX() : 0.0;
    }

    /** Left/right distance from robot to tag face (meters) */
    public double getY() {
        return tagToRobot != null ? tagToRobot.getY() : 0.0;
    }

    /** Robot yaw relative to tag (radians) */
    public double getYawRad() {
        if (tagToRobot == null) return 0.0;
        return MathUtil.angleModulus(tagToRobot.getRotation().getZ() - Math.PI);
    }


    public double getDistance() {
        return tagToRobot != null ? Math.hypot(tagToRobot.getX(),tagToRobot.getY()): 0.0;
    }
    

    /** Current vision measurement standard deviations */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return visionStdDevs;
    }

    double shooterHeight = 0.508;
    double phi = Math.toRadians(70);


    public double rpmFromDistance(double distance) {
        double y = 1.8288 - shooterHeight; // target height minus shooter height

        double shooterVelocity = Math.sqrt(
            (9.807 * distance * distance) /
            (2 * Math.cos(phi) * Math.cos(phi) * (distance * Math.tan(phi) + shooterHeight - y))
        );

        // Empirical drag correction — increases with distance
        double dragFactor = (1 + 0.015 * distance) * 1.04;
        shooterVelocity *= dragFactor;
        double wheelRadius = 0.0508; // meters (2 inches)
        double c = 1.0;
        return c * (shooterVelocity * 60.0) / (2.0 * Math.PI * wheelRadius) / 100;
    }


    
    
    

    
    
}
