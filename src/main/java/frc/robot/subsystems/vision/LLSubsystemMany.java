package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.List;

public class LLSubsystemMany extends VisionGeneral implements VisionIO {

    // *Initialize variables
    private final CommandSwerveDrivetrain drivetrain;

    private final HashMap<Integer, Pose2d> tagposes = new HashMap<>();
    private final HashMap<Integer, Double> tagambiguities = new HashMap<>();
    private final Debouncer alignDebouncer = new Debouncer(0.1, DebounceType.kBoth);

    // |Reset at top of periodic
    private double yawDeg;
    private double yawRateDegPerSec;
    private double omegaRps;
    private SwerveDrivetrain.SwerveDriveState driveState;

    public boolean hubPassed = false; // whether we've ever seen the hub (used to determine whether we should trust the alliance color and cached hub pose)
    public boolean hasSeededPose = false;
    public Pose2d cachedHubPose = null;
    public boolean hubLogged = false;

    // *Pose of the robot, wrapped in latestEstimate, as well as other logged variables
    private Pose2d estimatedRobotPose;
    private PoseEstimate latestEstimate;
    private double totalTimestamp;
    private double totalLatency;
    private int totalTags;
    private double totalTagSpan;
    private double totalTagDist;
    private double totalTagArea;
    private List<RawFiducial[]> allCameraRawFiducials = new ArrayList<>();

    // *Vision standard deviations
    private static final double BASE_XY_STD_DEV     = 0.5;
    private static final double THETA_STD_DEV        = 9999.0;
    private static final double MAX_AMBIGUITY        = 0.9;
    private static final double MAX_LATENCY_SECONDS  = 0.25;
    private static final double MAX_OMEGA_RPS        = 2.0;
    private static final double FIELD_MAX_X          = Constants.FIELD_MAX_X;
    private static final double FIELD_MAX_Y          = Constants.FIELD_MAX_Y;

    //private Matrix<N3, N1> stdDevs = VecBuilder.fill(9999.0, 9999.0, 9999.0);

    // *Miscellaneous
    private int contributingCameras = 0;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final String[] llCameras;

    // !In constructor, start a background thread
    public LLSubsystemMany(CommandSwerveDrivetrain drivetrain, String... llCameras) {
        this.drivetrain = drivetrain;
        this.llCameras  = llCameras;

        for (String cam : llCameras) {
            LimelightHelpers.setPipelineIndex(cam, 9);
            LimelightHelpers.SetIMUMode(cam, 0);
        }

        // // *Start background polling thread
        // Thread pollingThread = new Thread(() -> {
        //     while (!Thread.interrupted()) {
        //         PoseEstimate[] fresh = new PoseEstimate[llCameras.length];
        //         for (int i = 0; i < llCameras.length; i++) {
        //             String cam = llCameras[i];
        //             LimelightHelpers.SetRobotOrientation_NoFlush(cam, yawDeg, yawRateDegPerSec, 0, 0, 0, 0);
        //             fresh[i] = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam);
        //         }
        //         synchronized (measurementLock) {
        //             latestMeasurements = fresh;
        //         }
        //         try {
        //             Thread.sleep(50); // poll at ~20Hz
        //         } catch (InterruptedException e) {
        //             Thread.currentThread().interrupt();
        //         }
        //     }
        // });
        // pollingThread.setDaemon(true);
        // pollingThread.setName("LimelightPollingThread");
        // pollingThread.start();
    }

    // !Old constructor, without background thread (keep around for testing and fallback)
    // public LLSubsystemMany(CommandSwerveDrivetrain drivetrain, String... llCameras) {
    //     this.drivetrain = drivetrain;
    //     this.llCameras  = llCameras;

    //     for (String cam : llCameras) {
    //         LimelightHelpers.setPipelineIndex(cam, 9);
    //         LimelightHelpers.SetIMUMode(cam, 0);

    //         //LimelightHelpers.SetFiducialIDFiltersOverride(cam, new int[]{});
    //     }
    // }

    // *Periodic
    @Override
    public void periodic() {
        refreshAllianceCache();

        //Stop logging when the hubpose is seeded
        if (!hubLogged) {
            Logger.recordOutput("HubPassed", hubPassed);
            Logger.recordOutput("CachedHubPose", cachedHubPose);
            hubLogged = hubPassed;
        }
        // Logger.recordOutput("Vision/PeriodicRunning", true);
        // Logger.recordOutput("Vision/PeriodicTimestamp", Timer.getFPGATimestamp());

        driveState = drivetrain.getState();

        yawDeg = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        yawRateDegPerSec = Math.toDegrees(driveState.Speeds.omegaRadiansPerSecond);
        omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        contributingCameras = 0;

        estimatedRobotPose = null;
        latestEstimate     = null;
        //stdDevs            = VecBuilder.fill(9999.0, 9999.0, 9999.0);
        totalTimestamp     = 0.0;
        totalLatency       = 0.0;
        totalTags          = 0;
        totalTagSpan       = 0.0;
        totalTagDist       = 0.0;
        totalTagArea       = 0.0;
        allCameraRawFiducials.clear();
        tagposes.clear();
        tagambiguities.clear();


        for (int i = 0; i < llCameras.length; i++) {
            String cam = llCameras[i];
            LimelightHelpers.SetRobotOrientation(cam, yawDeg, yawRateDegPerSec, 0, 0, 0, 0);
            PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam);

            boolean camValid = isEstimateValid(llMeasurement, yawDeg);


            if (llMeasurement == null) {
                // Logger.recordOutput("Vision/" + cam + "/Valid", false);
                // Logger.recordOutput("Vision/" + cam + "/RawPose", Pose2d.kZero);
                // Logger.recordOutput("Vision/" + cam + "/TagIds", new int[0]);
                // Logger.recordOutput("Vision/" + cam + "/TagPoses", new Pose2d[0]);
                continue;
            }

            //Logger.recordOutput("Vision/" + cam + "/Valid", camValid);
            //Logger.recordOutput("Vision/" + cam + "/RawPose", llMeasurement.pose);

            RawFiducial[] fiducials = llMeasurement.rawFiducials != null
                ? llMeasurement.rawFiducials
                : new RawFiducial[0];

            // int[] tagIds = Arrays.stream(fiducials)
            //     .mapToInt(f -> f.id)
            //     .toArray();
            //Logger.recordOutput("Vision/" + cam + "/TagIds", tagIds);

            // !New version (faster since it pulls directly from RawFiducial)
            // *Fuse vision pose estimates to drivetrain pose estimate
            if (camValid) {
                //Matrix<N3, N1> camStdDevs = VecBuilder.fill(1, 1, 9999.0);
                //calculateStdDevs(llMeasurement);

                if (!hasSeededPose) {
                    drivetrain.resetPose(llMeasurement.pose);
                    hasSeededPose = true;
                }

                drivetrain.addVisionMeasurement(
                    llMeasurement.pose,
                    Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds)
                    //camStdDevs
                );

                contributingCameras++;
                totalTimestamp += llMeasurement.timestampSeconds;
                totalLatency   += Timer.getFPGATimestamp() - llMeasurement.timestampSeconds;
                totalTagSpan   += llMeasurement.tagCount > 0 ? llMeasurement.tagSpan : 0.0;
                totalTagDist   += llMeasurement.tagCount > 0 ? llMeasurement.avgTagDist : 0.0;
                totalTagArea   += llMeasurement.tagCount > 0 ? llMeasurement.avgTagArea : 0.0;
                allCameraRawFiducials.add(fiducials);

                for (RawFiducial f : fiducials) {
                    double x = f.distToRobot * Math.cos(Math.toRadians(f.txnc)) * Math.cos(Math.toRadians(f.tync));
                    double y = f.distToRobot * Math.sin(Math.toRadians(f.txnc)) * Math.cos(Math.toRadians(f.tync));

                    if (!tagambiguities.containsKey(f.id) || f.ambiguity < tagambiguities.get(f.id)) {
                        tagambiguities.put(f.id, f.ambiguity);
                        tagposes.put(f.id, new Pose2d(x, y, new Rotation2d(Math.toRadians(f.txnc))));
                    }
                }
            }

            // *Tag positions
            // Pose2d[] tagPoses = Arrays.stream(fiducials)
            //     .filter(f -> tagposes.containsKey(f.id))
            //     .map(f -> { Transform2d t = tagposes.get(f.id); return new Pose2d(t.getX(), t.getY(), t.getRotation()); })
            //     .toArray(Pose2d[]::new);
            //Logger.recordOutput("Vision/" + cam + "/TagPoses", tagPoses);
        }

        estimatedRobotPose = driveState.Pose;
        RawFiducial[] tagsUsed = totalTagsUsed(allCameraRawFiducials);
        totalTags = tagsUsed.length;

        latestEstimate = new PoseEstimate(
            estimatedRobotPose,
            contributingCameras > 0 ? totalTimestamp / contributingCameras : 0.0,
            contributingCameras > 0 ? totalLatency   / contributingCameras : 0.0,
            totalTags,
            contributingCameras > 0 ? totalTagSpan   / contributingCameras : 0.0,
            contributingCameras > 0 ? totalTagDist   / contributingCameras : 0.0,
            contributingCameras > 0 ? totalTagArea   / contributingCameras : 0.0,
            tagsUsed,
            true
        );

        Logger.recordOutput("Vision/PoseEstimate", estimatedRobotPose);
        Logger.recordOutput("Vision/IsAlignedToHub", isAlignedToHub());


        updateInputs(inputs);
        Logger.processInputs("LoggedVision", inputs);
    }

    // *Validation
    private boolean isEstimateValid(PoseEstimate estimate, double heading) {
        if (estimate == null || estimate.tagCount == 0) return false;

        double ageSeconds = Timer.getFPGATimestamp() - estimate.timestampSeconds;
        Pose2d pose = estimate.pose;

        if (ageSeconds > MAX_LATENCY_SECONDS) return false;
        if (Math.abs(omegaRps) > MAX_OMEGA_RPS) return false;
        if (pose.getX() < 0 || pose.getX() > FIELD_MAX_X) return false;
        if (pose.getY() < 0 || pose.getY() > FIELD_MAX_Y) return false;

        double visionHeading = estimate.pose.getRotation().getDegrees();
        double headingError = Math.abs(MathUtil.inputModulus(visionHeading - heading, -180, 180));
        if (headingError > 90) return false;

        return true;
    }

    // *Vision STDs
    private Matrix<N3, N1> calculateStdDevs(PoseEstimate estimate) {
        if (estimate == null || estimate.tagCount == 0) return VecBuilder.fill(9999.0, 9999.0, 9999.0);

        double xyStdDev = BASE_XY_STD_DEV;

        xyStdDev /= (0.35 * estimate.tagCount + 0.65 * estimate.avgTagDist);
        xyStdDev *= Math.pow(estimate.avgTagDist, 2);

        if (estimate.rawFiducials != null) {
            double maxAmbiguity = 0;
            for (RawFiducial tag : estimate.rawFiducials) {
                maxAmbiguity = Math.max(maxAmbiguity, tag.ambiguity);
            }
            if (maxAmbiguity > MAX_AMBIGUITY) return VecBuilder.fill(9999.0, 9999.0, 9999.0);
            xyStdDev *= (1.0 + maxAmbiguity * 2.0);
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, THETA_STD_DEV);
    }

    // *Gets the total amount of tags used in vision pose estimate. Tags seen more times = lower ambiguity
    private RawFiducial[] totalTagsUsed(List<RawFiducial[]> allTags) {
        HashMap<Integer, RawFiducial> best = new HashMap<>();
        for (RawFiducial[] arr : allTags) {
            if (arr == null) continue;
            for (RawFiducial f : arr) {
                RawFiducial existing = best.get(f.id);
                if (existing == null || f.ambiguity < existing.ambiguity) {
                    best.put(f.id, f);
                }
            }
        }
        return best.values().toArray(new RawFiducial[0]);
    }

    // *Getters for overall info
    public Pose2d  getPose()                        { return estimatedRobotPose; }
    public double  getX()                           { return estimatedRobotPose != null ? estimatedRobotPose.getX() : 0.0; }
    public double  getY()                           { return estimatedRobotPose != null ? estimatedRobotPose.getY() : 0.0; }
    public double  getRobotYawDeg()                 { return estimatedRobotPose != null ? estimatedRobotPose.getRotation().getDegrees() : 0.0; }
    public double  getRobotYawRad()                 { return estimatedRobotPose != null ? estimatedRobotPose.getRotation().getRadians() : 0.0; }
    public int     getTagCount()                    { return latestEstimate != null ? latestEstimate.tagCount : 0; }
    public double  getAvgTagDist()                  { return latestEstimate != null ? latestEstimate.avgTagDist : 0.0; }
    public double  getAvgTagArea()                  { return latestEstimate != null ? latestEstimate.avgTagArea : 0.0; }
    public boolean hasTargets()                     { return !tagposes.isEmpty(); }
    public Pose2d getCachedHubPose()                { return cachedHubPose; }

    // *Getters for specific tags
    // !These return 0 or false if the tag isn't currently visible, so be sure to check hasTarget() first if you want to avoid ambiguity with a real measurement of 0!
    public boolean hasTarget(int id)                { return tagposes.containsKey(id); }
    public double getX(int id)                      { return hasTarget(id) ? tagposes.get(id).getX() : 0.0; }
    public double getY(int id)                      { return hasTarget(id) ? tagposes.get(id).getY() : 0.0; }
    public double getYawDeg(int id)                 { return hasTarget(id) ? tagposes.get(id).getRotation().getDegrees() : 0.0; }
    public double getYawRad(int id)                 { return hasTarget(id) ? tagposes.get(id).getRotation().getRadians() : 0.0; }
    public Transform2d getTransformToTag(int id)    { return hasTarget(id) ? tagposes.get(id).minus(Pose2d.kZero) : Transform2d.kZero; }

    // *Setters (for all cams)
    public void setPipelineAll(int pipeline) {
        for (String cam : llCameras) LimelightHelpers.setPipelineIndex(cam, pipeline);
    }

    public void setIMUModeAll(int mode) {
        for (String cam : llCameras) LimelightHelpers.SetIMUMode(cam, mode);
    }

    public void setRobotOrientationAll(double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
        for (String cam : llCameras) LimelightHelpers.SetRobotOrientation(cam, yaw, yawRate, pitch, pitchRate, roll, rollRate);
    }

    public void SetFiducialIDFiltersOverrideAll(int[] ids) {
        for (String cam : llCameras) LimelightHelpers.SetFiducialIDFiltersOverride(cam, ids);
    }

    // *Distance utilities
    public double getDistanceToTarget(Pose2d targetPose) {
        if (targetPose == null) return -1.0;

        return estimatedRobotPose
            .getTranslation()
            .getDistance(targetPose.getTranslation());
    }

    public double getYawToTarget(Pose2d targetPose) {
        if (estimatedRobotPose == null) return 0.0;
        return Math.atan2(
            targetPose.getY() - estimatedRobotPose.getY(),
            targetPose.getX() - estimatedRobotPose.getX()
        );
    }
    
    public double getDistanceToTag(int tagId) {
        if (latestEstimate == null || latestEstimate.rawFiducials == null) return -1.0;
        for (RawFiducial fiducial : latestEstimate.rawFiducials) {
            if (fiducial.id == tagId) return fiducial.distToRobot;
        }
        return -1.0;
    }

    // *Returns distance to the closest visible tag
    public double getDistance() {
        if (latestEstimate == null || latestEstimate.rawFiducials == null || latestEstimate.rawFiducials.length == 0) return -1.0;

        RawFiducial closest = null;
        double minDist = Double.MAX_VALUE;

        for (RawFiducial fiducial : latestEstimate.rawFiducials) {
            if (fiducial.distToRobot < minDist) {
                minDist = fiducial.distToRobot;
                closest = fiducial;
            }
        }

        return closest != null ? closest.distToRobot : -1.0;
    }

    // *Returns the best available distance to a target — raw tag distance first, pose-based fallback
    public double getBestDistanceToTarget(int tagId, Pose2d targetPose) {
        double tagDist = getDistanceToTag(tagId);
        if (tagDist > 0) return tagDist;
        return getDistanceToTarget(targetPose);
    }

    // *Returns the best available distance to the hub
    public double getBestDistanceToHub() {
        if (cachedHubPose == null) return -1.0;

        double tagDist = getDistanceToTag(VisionConstants.getMiddleTagId());
        if (tagDist > 0) return tagDist;

        return getDistanceToTarget(cachedHubPose);
    }

    // *Returns true if both the vision pose and drivetrain pose agree that we're within the HUB_ALIGN_TOLERANCE_DEG of facing the hub, and that the two methods agree with each other (to avoid trusting a potentially bad vision estimate that just happens to be aligned)
    public boolean isAlignedToHub() {
        if (cachedHubPose == null) return false;
        Pose2d robotPose = driveState.Pose;  // use the already-fetched driveState (see fix below)

        double angleToHub = Math.toDegrees(Math.atan2(
            cachedHubPose.getY() - robotPose.getY(),
            cachedHubPose.getX() - robotPose.getX()
        ));

        double yawError = MathUtil.inputModulus(
            angleToHub - robotPose.getRotation().getDegrees(),
            -180, 180
        );

        return alignDebouncer.calculate(Math.abs(yawError) <= VisionConstants.HUB_ALIGN_TOLERANCE_DEG);
    }

    // *Refreshes cached hub pose until the alliance isn't null
    private void refreshAllianceCache() {
        if (cachedHubPose != null) return;
        if (DriverStation.getAlliance().isEmpty()) return;
        hubPassed = true;

        DriverStation.getAlliance().ifPresent(alliance -> {
            cachedHubPose = VisionConstants.getHubPose(alliance);
        });
    }
    
    // *VisionIO implementation
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.hasTarget = hasTargets();
        int bestId = -1;
        double leastAmbiguity = Double.POSITIVE_INFINITY;

        for (Map.Entry<Integer, Double> fiducial : tagambiguities.entrySet()) {
            if (fiducial.getValue() < leastAmbiguity) {
                leastAmbiguity = fiducial.getValue();
                bestId = fiducial.getKey();
            }
        }

        inputs.targetId = bestId;

        inputs.hasTagTransform = inputs.hasTarget;
        // inputs.tagToRobotX    = inputs.hasTagTransform ? getX(bestId) : 0.0;
        // inputs.tagToRobotY    = inputs.hasTagTransform ? getY(bestId) : 0.0;
        // inputs.tagToRobotZ    = 0.0;
        // inputs.tagToRobotRotZ = inputs.hasTagTransform ? getYawRad(bestId) : 0.0;

        // *Dont use streams, they are quite resource heavy.
        int[] ids = new int[tagposes.size()];
        int it = 0;
        for (Integer fiducial : tagposes.keySet()) {
            ids[it] = fiducial;
            it++;
        }

        inputs.visibleTagIds   = ids;
        inputs.visibleTagPoses = tagposes.values().toArray(Pose2d[]::new);
        // inputs.allTagToRobotX    = new double[ids.length];
        // inputs.allTagToRobotY    = new double[ids.length];
        // inputs.allTagToRobotZ    = new double[ids.length];
        // inputs.allTagToRobotRotZ = new double[ids.length];

        // for (int i = 0; i < ids.length; i++) {
        //     int id = ids[i];
        //     inputs.allTagToRobotX[i]    = getX(id);
        //     inputs.allTagToRobotY[i]    = getY(id);
        //     inputs.allTagToRobotRotZ[i] = getYawRad(id);
        // }


        inputs.hasEstimatedPose       = estimatedRobotPose != null;
        inputs.estimatedPose          = estimatedRobotPose != null ? estimatedRobotPose : Pose2d.kZero;
        // inputs.estimatedPoseTimestamp = latestEstimate != null ? latestEstimate.timestampSeconds : 0.0;
        // inputs.numTagsUsed            = latestEstimate != null ? latestEstimate.tagCount : 0;
        // inputs.avgTagDistMeters       = latestEstimate != null ? latestEstimate.avgTagDist : 0.0;
        //inputs.distanceToHub = getBestDistanceToHub();

        //Matrix<N3, N1> stdDevMatrix = stdDevs;
        //inputs.visionStdDevs = new double[]{stdDevMatrix.get(0, 0), stdDevMatrix.get(1, 0), stdDevMatrix.get(2, 0)};
    }
}

// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%@@@%@%%@%%%%%%%%%%%%%%%%@@@@@@@@@@@@@@@@@@@@@%@@@@@@@@@@@@%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%@%%@%@@@@@@@@@%%%%%%%%
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@@@@@@@@@@%%@%%%%%%%%%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@@@%%%%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%@@%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@%%%@@@@@@@@%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@%%%%
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@@%%%%%%%%%%%@%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%@@@@@@@@@%%%%%%%%%%%%%%%%%%%%@@@@%%%@@%%%%@@@%%%%%%%@@@@@%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@@%%%%@@@%%%@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%@@%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@@@%%@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%@@%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#####
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##########
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##################
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%@%@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%###########%%%%%%%%%%%%%%%%#%%##########################%##
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#############%%%%%%%%%%######################%%%%%%%%%%%%%%%%##
// %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@@@@@@@@@@@%%%%%%@@@@@@@@@@@@%%%%%@@@@@%%%%%%%%%%%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###################%%%%%%%##%####%%%%#####%##%%%%%%%%%%%%%%%%%###%##
// @@%%@@@@@@%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%@@%@@@@@@@@@@%%%@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%####################%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#####%
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#########################%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%####
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%##%########*##*#########%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#######
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%########################%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%########
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%##%%%%%%%%%##########%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%%%##########*
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@@@@@@%%%%%%%%%@%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%@@@%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%##########***
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%@%%%%%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%@@@%%%%%%%%%%%%%%@%%@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%@@@@@%%@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%#%###########*****
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%@@@%%%@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@@@@@@@@@@@@@@@%%%%%%%%%%%%@@@@@@@@@@@%@@%%%@@@@%@%@%%@@@@%%%@@@@@@%%%@%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###############**********
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%@%@@@@%%%%%%@@@@@@@@%%%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@%%%%%%%%%%%%%%%%%%%%@@@@@%%@@@@@@%%%@@@@@@@@@@@@@@@@%%@%%%%%%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%################%%%##########******************
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%@@@@@@%@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@%%%@%%%%%%%%%%@%%%%%%%@@@@@@@@@@@@@@@@@@@@@%@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###########%%%%%%%%%%%%%%%%%%%%%%%%%%#####%##%#################################******************
// @@@@@@@@@@@@@@@@@@@@@@@@@@%@@%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@@@@@@@@@@@@@@@%@@%@@@@@@@@@@@@@%%%%@%@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#####%##%#####%#%%%%%%%%%%%%%%%%%%%%%%%%########%########%###%###############********************+***
// @@@@@@@@@@@@@@@@@@@@%%%@@@@%%%@%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%@%%%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@%%%%%%%@@@@@@@@@@@@@@@%%%@%@@%%%%@@@@%@@@@@@@@@@@@@@@@%%@@@@@@@@@@@@@@@@%%@%%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%########%%%%%%%%%%%%%%%%%%%%%%%%%%%###########################*#*****************************++******
// @@@@@@@@@@@@%%%%%%%@%%%@@%@%%@%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@@@%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%@@%@@@@@@@@@@@@@@@@@@@%%@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###%#%##%##%#%#%%%%%%%%%%%%%%%%%%%%%%%########################*#********************************************
// @@@@@@@@@@@@@@%@%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%@@@@@@@@@@@@@@@@@%%@@@@@%%%%%%%%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%%%%%%%%%%%%%%%%%%%%%%%%##%##%##%####################*************************************************
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%@@%@@@@@@@@@@@@@%%%%@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%#%####%%%###################**************************************************
// =-=++**#%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%@@@@%%@@@@@@@@@%%%%%@@@@@@@@@@%%%%%%%%%%%%%%%%@@@@@@@@@@@@@@@@@@@%@%%%%%%%%%%%%%%%%%%%%%%%%########%##%##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#####%######################***************************************************
// "===+#**=-::=*****+@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@@@%%%%%%@@@@@@@@@%%%%%%%%%%%%%%%%%@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%#%##%#######%##########################%#####%%%#############################*#******************************************************
// "*****+**++=+**+**+=++++++*+===+%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%@@@@%%%%%%%%@@@@%%%%%%%%%%%%%%%%%%@%%%%%%%%%%%%%%@@@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%########################################################################****************************************************************
// ====-++*+*+****=+==.=****%#****+==*+*+===*#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%@%@@@@@@@@%%%@%%%%%%%%%%%%@@@@@@@@@%%%%%%%%@@@@@@%%%%%%%%%%@@@%%@@@@%%%@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%#########################%%%%#%##%###########################################**************************************************++****+*++
// ==++*==++*+*+*****+*+*+***+*+*#+**=+#**#*******+*+***#%%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@%%%@@@@@%@%@%%%%@@%%%%%%%%%%%%%%%%%%%%%%@@@@%%%%@%%%@%%%%@@@@@@@@@%%@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###%##%%#%%%%%%%%%%%%%%%%%%%%%%%%%%%#############################*#***##########*#*********************************************+****+*++++++++++
// @%**%*====*+*+**-**=***==---=-**+++++#%*--:++:==*--=-*+**+=+==**+=*+=+=+==****+***+***======++++*+*+*=+=+*%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%@%%%%%%%%%%%%%%%%%%%%%%%%@@@@@@@%%%%%%%%%%%%%%@@%%%%%%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%################################*#*****************************************************************************
// %%*#%%%%%%#%%#%#*##*%*+**++#*+++=*+***+**++**+*++-:-====++=+++=+=+**==+**+**+=:-==+=+*+*===+*+++==::======-:-:=+*************++++******###%@@@@@@@@@@@@@@@%@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@@@%%@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%#%###############################*#########****************************************************************
// "**=*%#%%###%%%%%%###%*%*######****=*++**+*+***=****++--:--:=*====+**+====*=+=+=----=====+====+*+======::..:::..:==::::=====++=+++++-:::=::.-::=:*@@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#####%%##########################****###*******************************************************
// ---=+*+*#*#**#%#*%***##%%#**#*****+**==+=-===++****+*+*+**+*==-:===-==--===++=*=+-====-=+-==::--=++=====+==-:::-::.:.:::-:.::..::-==+=::-:--==::-+:-@@%%@%**%%%%%%%@@@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#####%#%%##%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%#%%%%%%%%%%%%##%%##################**************************************++****+**************************
// "**+******#**#%*##+****#*%%%%%%%%#*##***###*++#+=-+==+**+*+#==+*+**+++**+*==++++=*+***=++*+*****+*#+**====:=-=+++=: :*=:::-::.::=.:*=.:::::::.:===::::-==****=-::==-:.-=::::%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%##%#%####%####%%%%%%%#%%##########%#%%######%%#%%%%%%%%#%%%#%%%####%##%%%#####%%%#%##########################*##*******************************+**++++++*+*+++***************************
// ====++*=+=******=***++*******=+***%%%#*****#**+++=-=+*+=**+***+*=+::++++*+=*+=+-=++++++*+**+===*+*++**===+++=***+=+=-:::.::-:.:.:=*=:...:-.::::--=-=::.:=#%%=:=::-:::::::::::=::%%%%%%%%%%%%%%%%%%%%%%%#%##%%%%%%%%#%#%%##%%#%%#%%%%##%%%%#######################################################%%%#################*********************************************+***+++**++*++*++*****************************
// +**===+=*++===...-=**********+=*+**-=*==+====+++++#***++*=+*++:++*++++**+=+++*+=+*++***#*+++===+--+====*+*+*==+===:=**+:. .:.::::=:..:::.==::.:::--=.::::+-:::=::--:::.:::::..:::+*%%%%%%%%%%%%%%%%#%###%#%%###%%######%%##%%%%%#%%%%%%%#%%%%#%%#################################################################*********************************************+++*+**+**+********+******************************
// +***++=++++*+==-===*=-=++************+*===**=***#+***=+=+*==+#**#++**=+*+++=*+*=====+*******+**-+==*#==***==++=+*+*==::+*---:.:::-..:..=-:.::...:::=.....::::::::::-::.:::::.::.:=:-#%%%%%%%%%%%%#%%%%%%%%%%%%%%%%%%%%%#%%#%##%%#%%%##%%%%#####%###################################################**####*##*##**#*##********************************************+*+*****++********************#########%%%%####
// +====*====+++*=+**==++*+**====+=+#=#**=+**************#****+++===***++**===-:==+=+++=*==+*=**=*==****=++=+==*++=-=+=:-=++++*++:.:: :==:+=::.:..:+::+.:.:::::::::::-:::::::::.::.:..::+++==++*+==+*+*=-*#%%%%%%%%%%%%%%%%%%##%%##%###%%#####################################################**###*####*****#****#####*###**#********************************************************##############%%%%%%%%%%%%%%%
// -===++=#******==+===+=====.-:==.--:.====**+:-=:-=**+++*+*******=++=++=+*+++++++*+++:=:====++*++******+===**+==+==-::::=:+=:==*=::::*: ::::=: :=.-:-:.:.:.::::::::-:-::::::::.::.::::.:*==:*+::::::::::-:%%%%%%%%%%%%%#######%##########%############################################**********************#**###**#**#**********########################***#####################%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// "*=**=-=:::::=:==:=-*===========-.+=++===+***+=*#+=::--.==+-++*+*+++==+*+==+*++=+**====+-=+=+:-:=%==++-===+=+-+==-=.::-:::::-:.-=::.:::::=::::::::::......::.::::-:::::::::::::::::::::===+=:..:::::.:::%%%%%%%%%%%##%%##%%%##%##########%##############################################************###********########*****#########################%%#%###%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// -:-:#-:.--===+:-=++=+=*=++=+**+=****+*+*=+=*=++=+=:=**====++==++*-*+=*++++-=*==-:-+:-+=+**++#*#*+*-+=*+*:=*+****+=++=    . :: .. ..::.-::.:..:..::::......::.:-:::::::.-:::::::.:::..:-::::-:..:.::::::.%%%%%%%%%%%#%%%%#%######%%%##%%#######%######################################**##*##****#############################*##*#########%######%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//     .:.:::..:::::-=-+:*=-:-======-:.:::::--:.::-===-::-:==*++++***+=++***+***=****+**==:=-+*+**==+++=-*++**++*=+++**+*=+==---:=- .:...-::.: :..:.::-:........:::::::::::::::.:::::::....:-::::.::::::::=%%%%%%%%%%%%%%%%%%##%#%%%######################################################%%%%%#%%%#######################%##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// ++=:=+====+===::--+-::--..::..:..::=*:     ..:   ..:. . ..: --....-==*++*=*%#**+***+**#++**###=-=***++***++**+****+*+=+**++=*+*+.....::-:-:.:..:..::..:......::::::::::::::.:..:.:::.::.:.::::.::::::::=:%%%%%%%%%%%%%%#%#%%#################%##########################%#########%##%%#%%%%#%%%%%%%%%%#%%%#%%%%%####%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%##%%%@%%%#+*=*=-:=-:-===-:-==::-=-:-===+*==*=::++::=.:-.-:.=+===:=:.::====. .:=-.:-+#+*=--***##**#*****+****##*+#**********++***+*++-- :::...:.:........::::::::::::::::.::..::.::...::::.::::::::-.%%%%%%%%%#%%%#%%##%##%#%##%##%##%########################%%%##%#%%%##%%%%%%##%%##%%%%#%%%%%%%%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// "%#%%%%%%%%#%%%%#%@%#%%%%%%@@%%@@@@%%%%%%%##%%@%%%%%*%%%%%##***##+====+====-==*:-=++**+%*:=*****#**+*+*+**+*+****+*#*%+**%+**%+***#+***#*-==*++ =:.:::. ..:...:::::::.::::::.:.::.::.:::::::::..::::::::.%%%%%%%%%%%%%#%#%%%#%%#%%%%%######%#############%#%%%%#%%%%#%%%%%%%%%%%#%#%###%####%%%%%%%##%####%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%##%%%%%%%#%%#%##%@%%%%%%%%%%%@%%%%%%@%%%%%%%%@%%%@%%@%%%@%%%@%%@@@@@%@#*%%%##*%*#*=+*+*#**+*=-+==+*==-****#*##*=*+=******#+*****+*****+=+=+*+-::...::.:::::::-:::-:-::::::::::::::::::::..::::::.:.%%%%%%%%%%%%%%#%%%##%##%%##%%#%#####%###%%##%%%%%%%%####%%%%%%#%%#%%%#%%%%%%%%#%%%%%%%%%%%%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%%#%%%%%%%%%%%%%%@%%%@%%%#%%*%%@@@@%@@%%%#%@@@@@@%@@@@@@@@@@@@@@#+#@%%***##%%*=#%#+*+#*#%+#%***##=*+##***+#+*****#=*=====*****+=*==-:-*==+=:...:::::::::--:::--::::::---::::::.:.:..:::::::.::%%%%%%%%%%%%%%%%%#%%%%%%#%%####%####%%%######%#%###%%%%%%%######%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// @%@@@%%%#%%%%%@@%%%%#%#%**##%##%#*#*##**=+++=+==%%%%##%*##*****#**#******#*+#*#=*--====*+****++**#**#**+***+**=*==:.:======+.:-::-=:.::-=+-==::=:=====+*=:.....:::--:----:--:::::::::::::..:::::.:::::.:-%#%####%###%##%#####################################*#**################*############*###########################################%%%%%%%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// "#%#%**%+==*+**%**#*%*#*%*#%+%******+***%%%%*#*%%*#**+*####*=#=*#*++**#%%%**+=****++*@++%%*%==+==+=-=:-*#%--+*====-:=:--+=*+==.=:.:-:.:::=--::-+:*=*++***==.::::---::-=-=--:-=:---:=::.::..:-::.:.::::::-%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%%###############################*###*####**#*#**##*#*#*************************************************************+*****+***+++**++++**+++*+*****************
// "****%#=****%********%*##*#*##**+*=+*+=+*+=*#**+*%*#*********%#*******==+%+**+#+-.=++*#*=+=+*+-=+==*+%++*#=+++****+**:::::.::===---:-=+===+**#%****+*##%%#%#=-::=--------===-==::=:---::::::::::::::..:::%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%#%%%#%%######%%#############################*########*#*##*##********#********##***********#*******************************++++*++**++*++*+++*++++++++++++++++++++
// "+**%##+*#***#==*=+%*+###+*+=++%#+#=**+##==+*#*##**-==*%**++#***==++=*++#=*+-+#+=:*-%%*=#+#+***##**++==*###===-:=***=*::. :::-:-:===-:-=*#%##%#+===::::-:::..:*%*+.:::-----=-==+=====--::-::-=--=++=*#++=%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%########%%###################*###*#*#*****#**#******************************************************#**###*####*###****#***********************
// %#***+**##-=***%****#==+*==+=***=##*+*++++*%**+*#=:-===*++-::::=--====+%++++==++-+*+=++*+-=*==***++*#*=*##*++=**=:-++=-+=:-:-:-..:=****===:=+*=:-:  :::::::..  :==:*+=:::=-:=======-::--:::..=:.::::-=:::%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%%%%%%%%%%#%%%%%%%%%%%%%%%%%%%%#%%#%%#############################*#####**##**********#*##****#*#***##**#******************************************************
// "*#*%##***#*=*+%%%%#%*##*=+*=*==*#=:=**#**%++=+*===#++#=-**+= =*:*-*%+=+=-===--=:--=-++*%+*+====+*=*=*==:======*-=::-.--=+---...-...==:.-=.:   -==  .::-:*=:.:-:.-.--==::===+=*+==-=-==:::::-:-.:::----==*%%%%%%@@@@@@@@@@%@@%%%@%%%%%%%%%%%%%%%%%%%%%%%@%%%%%%%%%%#####%%%#%%%#%###%#%#########**###%#######%###%#%#%##%%################################***#**#****##******#**********************************
// -==-:=+=====#***%*#-===:=****+**+=#=%***==++======*+====**++===-=@=:*=**#*+-::*==+++*#=*====++****%==:-::::-==::-=- .:::::====::....:-::=.    .. .   :--.:.. . .=::..::=:=-=====++--*:-----:=:::::::::::::-@%@@@%%%%%%%%@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%#%%%%%##%###%%%%##%##%%%%%%%%%%%#%%%%%%%###%%%#%#%##############%###############*#######################**#####*****************************
// --:::...-::+#*=-=+=:::+::----+---+--%::::-.::.:+.:::.:==--:=*=**==-:*=======*=:::=:.+=-+=*+*==*::==+:-+--.:-:-=::.:%:.-:.--.:-= : .:::.-:.. . ..... .+=:::  : :.:  : :-:==-*-=-:====:--=-==::-==.-+===++-++*@@@@@@@@%%@@@%%@%%%%@%@@@@@%%%%@@@@%%%%%%%%%%%%%%%%%%%%%%%%#%%%%%%%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%%%####**########*#######%############*#####**##***********##*#####*****##**#******************
// ==:. =*##*=**+====-=. -:::-++=:=:=::*-: .--=-::-::...:-:::=+=-***++**=:-==:-==:=*++++*%*=**#**==*+::=:.-:  :::..::-::::::::=.=-::::-:....::     . .:::::.:::-::: :  :.::::--=::----:::---+==:::-:.::::::-===*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#####%#%##########%####%#%#%%#%%%%#%#%%%%%##%%##############**#**######*##########******#*#*****
// -==+*+#**#+=-::=:-=:-=*.+-+#:+-:+.=:::.:=:::+-==-====-=*=-:=*%++-+=+===+*+**%*#%#*==::=*==*=*-=++==--:::  :::    . :. -::: .:-=:::-:..::...   .:- ::::::::=::..:.:...:  --:::-:::--::-::-:-==-:=::--=::-:--==:@%%%%@@%%%%%%%%@%%%%%%%%%%%%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%%%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%#############*######***#*##***###**##*######*###*#############****##******##***#***##**************
// .:-+===*=-=:::-:::++=++#+**#+=+*= ++=::-++===---:.:::::.:-=+*+*+**#+=#++*#====**-:---:--:.:.=.:*=====:-:::...   ..    :...:::::: .::::....:  .--::.-=.-:=%--: .+=::::::-:=:::::::::::::::::::-===-:::-::.:.:::::#@%%%%%%%%%%%%%%%%#%#%######%%#%##%%%%#%%%%%%%%%%%%%%%%%%%%%%%%#%######%###%###*########*########*#######*###*###*##**#*##***#*##*#######*****##*##*******##***#***#****************************
//  ::-=::-===::. ..::=-=+=++=-=**+=+*:*=:==..: ..--::.:.::::=-+:==+=:=**#**+:.:.:: .::: :=:--===*==:. .: :=:   . .     :::::.=::::  . .: .:.::  = ===::%*:::-#-  =: .. ... :--==:.:-====-=:-====-====:::-=...::::::::*@@@@%%%%%%%%%%%%%@%%%%#%#%%%%%%%%%%#%%%%%###%####%##%%%%%%%####%#%%%###%%%##%##%%##########%%%%%%%%%%%%%#%%###%%#%#%#%#%##%###############*###****##******###******************##******#####
// =:::...::::::::...:=*:::::::-**++**--:=+=:   .=*-::-=-+**+*+=-*=**++*++-. :..  :.      :: :  :. ::=*=.  .::.  ..       .::.::  :  : =.::::::    --.=#:.::::--::.....::::: :=--:=++==--==-::-======+*+-::::::::::.:.:=@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%%#%###%#%####%%###%%%%#%%%%%%%%%%%#%#%%%######%%%#%%#%#%%%%###%%%%####%%%%####%#############%####%#%%%###%######################*#####***##*#****#**
// ..  .-.  .-::-==::.-==: ...::=::=-=-*.-=+::-*--:=.*==*#=#++**+==+=:::==-==     : .  :== .. :::=-.:::+:.   ..   ..  ..:.. .::   ++=:.  .+.:=::   :: : ...::::-.@   ..::-::.:+-=-=++*+=...:..:::--=+==-:-=::-::::-:.=+@@%%%%%%%%%%%%%%%%%%%%#%%%#%%%%%%%%#%##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%##%%%##%%%%%%##%%%%%#%#####%%%#%%##%#%%#######%%###%#%##%#%#%%%#%%%%%%%#%%%#%%##%%%##%###%###%#%%##################
//    :  ::=:: :=.:*=   -:...:-:::::.:%#=+=:::::::-:.===+*==++=+==+-::..::::: :=-:-=:.::  :::    :=::==:.: ..     ...  ::..=:=:.-::=:=::: .::=.:  . ..=::....:.:.  ..     :+=:.----===#*+--::::.=--===-::::::::.... .#%%%%%%%%%%%%#%#%##%%%%%%%%%%%%#%%%%%%%%%%%%%%%#####%%%%##%%%##%%#%%%%#%%%%#%%%%##%%%%##%#%##%%%%%%#%%#####%%#%%#%%#%%%%%#%%%%%###%######%##%%######%#%%##%######################*############
// :*==+-=-:-===-.-==:-:-.:: *::::.-- @%:*:.: ::::.  --===::-=:-:*  .:.::::: .:-... :=-.: .  . :+**==++=-*=-    ...:..:     ::=.::::=:::::    .=  .:-=@*:..: :-:  :.      ..:  :-+::::==::-:=:-:=:::.:::...:::..=%@@%%%%%%%%%%%%%%%%#%%#%%#%%%%#%%####*#######*%########*####*######*#*########%%#%%########**####**#**#**############*####***##*##*########################*###*#****###*##*##**##*##*##*####***##
//  -:::.:-=-::..:#:+*====+-*--::+=***#% =--:::==-=====-==-:: :.  .-=: ...    .  :.::: :.--+:  :====:=-=--:=:::  . .: .: .  .. .:  .::-=  .--. .=.-.. .. . . ..+-......  .-.-+  :==::::::=+. :.::.:: .=:.+*##%##%%%%%%%#%%%%%%%%####%##%##%%#%#####################%%%#%%%%%%%%##%#%%%%%%%#%############*#*###########*##*####****#*****#******########*######%##%##%#*#####**##***###******###############*###**#*
// ==-:=.%#**-==+#*:+++*=+*=+--.====++%%=#*=::-:-:-:===:=-==:  :+.  :  ..   ...        .  ..: :=+-::.:::::.-:=   ..: :  . .::...- .  : ...::*   ++=      . ..==:.-::::.    .= -  .:::..:.  :. ::.--=%%%##*##%##%#%%%%%%%%%%%%%%####%%##%%%%%%%%%%%%#%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###%%%%%%%################%###########%#########%#####################%#####%############*#######**###*#*#*#***##******#*#****
// "=++#===*#=*%#*%:=###**##%+=+**:-#+*%*=#=-:::::::.  ..:: .   -.. :#:  .      :=:%:::  .  .-=-:::::.:::::=-*  ::. ::.::::.::...::.    .:..             . ..:=:=:=+=.      ::.  .    :++#**#*#**#*######%%%%%%%%%%%%%%%%#%#%%%%%##%%%%%%%%%%%%%%%##%#%%##%%%%#%#%%%%%%%%###%%%%#%#%%%%%%#%%%#%#%##%%#%%#%%####%####*####%####*########%##%########***####%%%##%%#######*####*##*####%####*###%##%##*****##*##*###*
// %=*%%%#%%%******=+**##%%%@%=%@@%****%%%*+:+.=== :.-  ..   :#... ::-**=*+:    ::  ===::-.*===+.=:::..:::=*=+=+- :*=:. ..     .:...:. . :::.         :::.:::- =-::.  . .  .:.  %%***+*****#**#*#######%%%#%%#%%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%%%%%%%%##%%%%%%%%%%%%%%%%%%%%%%#%%%%%%%%##%%%%########*###%####%%%#%%%%%%####################%####%#%#########%%###%%#########%###########*####
// #=##%%****-*=+=*=#++***+##%%#%%%%#%%%%%##*%*-*+==#: .-:#*@#%*#*%%%%%#%#**=**+**=* ====:-%=:=-===.:..:%=%...:===-: ..   .    .:::.::..::::-.   .:: ..::.::+**+-=.. .:.       :+++*****####%%###%%%%%%%#%%%%#%%%%%%%%%%%%%%%%%%%%@%@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%##%%%%%%%%%%%%%%%#%%%%%%%#%####%%%%%%%###%####%%########################*#***####*########%############*##########
// "=+=*+=*+##*****+%@%**%%%#%%@%@**%+%*#%%#*#%%*%*%%*#%@@@@@%%@@@@%%*%##*#**#@%%*%#%#%*=+=+=++-+=#-==+=:-:::.:===*=++-.:.:::::..    :.. .: ... .:     .  .===::::=:: :      ::.-+**###%#%%###%#%%%%%%%%%%%%%%%%%%%@%@%@@%@@@@@@@@%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%%%%%%%%%##%%%%%%%######%###*############*#*#*##*####**####*#*##**##**##*##**#**###****####*#######**#**#*#**
// "-*+==+=*#%#%*%##%=#%#%##%%%##*@*#****##%%%**+**+**#%%@%%@%%%#%@*%%@%***++*%***=+**%%@%#%%@%%%*@%#%+*+#+=.@=:: :....+=.....:: ::..:   .-..::   ..    ..:==:   =::*::         :##%%%##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@%%#%%%%%%%%%%%%%%%%%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%%%%#%%%%%%%%%%%%%%%%%%%%%##**#####*#####*##*##*##**#######***###*#####*#%######*###*##*##**######***##*#*****#*##****###*##########
// #=**%***#*#%%%###%*##*#*#++#**#=#+*:==*#=***+*=+==****%*@%%%%@#*%%##**%##**+=*:=:+-.-=- :=-=*+*==*=*@##@++===:--..-:--=-:..    . ..::   ... .     .  .  :*::    ..:=:    .   -###%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@%%%%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%#######%%%%%#%%%%%#%###**#**#******######*#######**#########*#*#####*####%%####%##############%########*#*#*###********#####*#*##
// %+**%#*%%***+#*=*#=+*:*#*=*=+***=-*-*+==**#=:::==:::=++#%#***%%*%#%%%#***%=:-=-==*=  ::-:..=:..-:* :*:= .==#====:: =:.=: =*:.:  ..-: :.  .:.              ...  ..  : .::..   -%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@%@%%%%@%%%%%%%%@%%@%%%%%%%%%%%%%%#%%%%%%%%%%%%%%%#%%#%%%%%%%#*#######*#***####**#*#########*###############*##**#**###*###*###########****###%###%##%%#%#%**#########%################******
// %**##%%%#*=%#*+++=+%=+*%#:+**-#==*+**==-+=*+=-.:=-==:-+:=**=*+***#***+*#%**-:-=:-:. .:: .     ..=:=.-:.   .-=#%%====::==#*+=.  ..::::::  ...  :          :.   :.:..:..-=   .  ##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@%%%%%%%%%%%%%%%#%%%%%%%%%%%%%%%%#%%%%%#%#%##%%%%%%%%%%%%%#%%%%#####%%####%##%%%%%%%##*######*####################**####*####*######**##**####################**#############%################
// "##**#**#%*===:=*+=+%=+*+==**:=%*#========*.::-===----:==:===-===*+%*+:-=== :..:-==. : . :  .+=-: ::- --:++-#=+=#+*#%**=+*+===-:::::--: .. .: :=          :..      =::       .*%#%%%%%%%%%%%%%%%%%%%%%%%%%%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%%%%%%%%%%%%%%%%%#%%%%%%%%%%%%%%%%%%%%%###**####%%%%%#####****##*#*##********#****************#*****#***#*####%%####*#***#####
// #%#****++*+===:*++=::=-=+-:-==::=+:**%+#=-*==++: ...::::=:----::+-==*=+*+-=:-=::=:==: -:::+:::..... .   ::.:=:==+****=****=++++-::::..::...  . :.                  .    .:.   #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@%@%%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%####*####***#*******####******************##**####*###*#*****#*************##*###**######*###
// "=++*%%=+#%=%*==-#==- =*=. :::  :++=::+::=+==-=-:  :..-===::.:==+*-=*+*=:=::==:.= :==-:-:..=:          :   -:-==-++#+#+-=+=**-::  -:..:.. :   *                    .:.  ..    -%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@%%%%@@%%@@@%@%@%%%#%%%@%%@%%%%%%%%%%%%%@%%%%%%%%%%%%%%%%%%%%%##%%%%%%#%#%%%%%%#%%%%%%%%%%%%#*##*###########*#####################*###*************##*###*********#####*##*******##*******##*#
// =%*%%++++#+**==*+***=++*==+*==: - **-:.:*+::+*:=-+=:=-:    :.=:-::-:=====--==:::  :-= .-==:=:       .         ::=++*=.:  ....====    .:. = :  =  =      .        .::::-:  .    *%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@@%@@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%%%%%##%##%%#%%%%%%%%%%#%%%%%%%%%%%%%%%%%#####%###################*######################*######%#####*##*##*#***#*#***#**#*#*#*#####************#
// %%%%*:****=*=*=***:%%+=+*:==:=-.:*= -+*::.=%++:::=+=:-=:+:::.= :=.:.:-:::-: -:====.. :::::+=-=: ++.   ..   ::: :+**+*# =      =    ..  .:=.   :. :             .    :  ...    ..%%%%%%%%%%%%%%%%%%%%%%%%%@%%%%%%%%%@%%%%%%#%%####%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%###%%###%#%%%%%%%%%%##%%%##*#**##%######*#**############%%#%#%########*###%###%#*######%####%#########%%%%%######*##*###*#########****#*#
// "#%*#++#+*+*=+**+#@#-=##++==**=******-*:-**==+::-++=.=:==-.:.. :=..:.--::::==:--:. : .:  =-==::  ::       :.-=:.=*+*-== .    ..:::. ..  ::.  .                  .:. .: :=     . %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%################***##*####*###*#*##################%%#%##%%##%%#*#######%%%###%#%######################*##########*#*
// @%*%*#%%**=%*##+**#-*:*%**=+*=*:*=*#%*- =#*:.::++: .= ::-==--=*=:.::==::-:-=%:.:       .:.::=.:: :. : .     . ::== :#=*:    :-=. ::.-:.:::=:.   .        :    ..   :.         ..%%%@%%%%%%%%%%%%%%%@@%%@%%%%@%%%%%%%%%%%%%%%%%%%%%%%%#%%%%%%%%#%%%#%%%%%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%####%%%%%%%#####%%%###*##%#****###%#####***#%########%%%%%###*%#*######**###*****###*#*#####%###*##############%##*#
// @@%%#%%+-%*====**##:*.*::==:=-=*=:-:::-=-:  .: -:**+ :=-..::-:.::.:.=:.. ..:::==:  .   . .=::-.:::-:.       .      .   -::   -:::.::.::.:::==.:. .        .  :   .:.:         ..:*%%%%%%%%%%%%%%%%%%%%%%%%%%%%@%@%%%%%%%%%%%%%%%%%%%%#%%%%%%%%%%%%%%%%%%%%%#%%%#%%%%%%%%%%#%%%%###%%##%##%%#####%%#########*##%%%#%##%%%%%%%%%%%%%#####*****########**#**######*****####*#*##**#**###****#****########****######
// %#%*%*%%+%*:*@*+##.=:#+.=*#.:--== *:.*.:::   =%=- :==-:::=::::-===: :::..   :.:::      =.  .:=:.:-=:::==             .       :: ..:.:: ::::.--: :..         ..   :.::::::.    :. :%%%%%%%%%%%%%%%%%%%%%%%%%#%%%@%%%%%%%%%%%%%%%%%%%%%##*#####%%%%%%%%%%#%%###%%%%%%%%%%%%%%%#%%%%%%%%%#%%%%%%##%#%%%%%%%%%%%%%%%%%%%%%##%##########%###******###*######**+***#****************########%%##***##**#**#*######*#*#
// "#=**%##%##*=*=+===++:--:.:: :-#::-=:. :..:: :=:::-: ::.::.====:-:..:. ::. .:.:==  -=:  :.: .::: .::..-::.              .     ..:::::::::::.-:-::::.       . ..::.:  :::::   ::-  :@%%%%%@%%@%%%%%%%%%%%%%%%%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%###%################%##%%#%%%%%#%##%#%#####%##########**###******************************************#*#***********####################
// "===*=+%#%##%%#:**:-..:.  . ..+  =.:.%=.   +.==#:*- . ..===::=::=-::::.. ..:::..:==:===-..   ..     .:...:..            :-.   :: :=:::::::=::--:-==:%      . .  :::  . :-:.=..::+ .%%%%%%@%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#*#########%##%%%%##%%#%#%%###*#**#*##########**###***#*######*################*####%#########%%###***###%####%#*###%%%%#%##***+********
// #+=%****#***#***%-.::...:   .: :*+.:=:---::::-. ..:  .:==. :+=::=---::=:..  ..:::..::-.:::. :::::: :.. . ...          : :.    -:::.   ::.::.:::.:::=:         ....:....: :::: ..:. *%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%##%%%##%%%%%########*#####*#########################*###############%##########****####**##*##*#######*#######%################*#*##**##***#*#*#######
// --+++%*===+=*+#+==+:  . ::*-   .::..  ..=-:-  .      :.    ::=::+=-:=:-       .    .-:-++===::.:.-...                    :    :::- :.  :. ..... :.:.:- .       .::.:: .   : -:.    -%%%%%%%%%%%%%%%%%@@%%%%%%%%%%%%#%%%%####%%%%%%%%%%%%%##%%%%%%##%%%%%%###%###%###%%%%%#%###%######%##%#%%%###%*#########***##**####%#%##*#####%######**********###******###*##*********#*#*##*##**#******##########*#########
// ::==*+--:-=+==***=:=.-::  =:  =        : .:.: --=+ .:- :.=:-===---::-:.-::.       ::.-::-=-:==:---=-:..            :.    .      ... :.    ...   .. :.:=.       :.    : =+=- .:.     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%%%%%%%%%#%%###%#%%%%#%###%%%###%#%%%%%%%%%%%#%%#%#%%%#%%##%%%%#%%%######%#%##*#########*#####********#####*###******#*##***#***##*##*#*#*##****########*********#*#*##****#####*##
// +*=:::.-:..=@=+%**%-=:: :=-===: :=.     :#:::.. :-   -:   .:::.  .   :. .:.:=   .. ...:  ..:-    .::::---.::::.     :...::.:=.  :  : .      . .   .. :.              .==:==  .      *%%%%%%###%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%##%*#%#%%##**#*###%%#%####%%#%%#%%%%%%%%%#########%%%####%#######*######%##*****#*#***####***#*##***#*#*###*****#*************#####*######**********##**********#*
// =+--=.:: :=+==*##*#*=: ::=:.-:::-:::.=:        .::      -:  :..:=-=:--:=-:::-           :.        .: :      ::.--=*.- :#:*=**. .-=-:.::.....      .:  -                :==-.        =%%%%%%%%#%%%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###%%%#%%%##%##%%%%%%#%%%#%%%%##%%%%%%%%%####%#%####%####%########**###%###########********#*****######**##*##*****####*#####**####********##*#*********#######*###***#*******
// ===+ ==:.:**==+***+*++*: .*++*=        ::     :        ..:=-  .-::=: ::--.=:   :::      .             ::        ..-  .  *::%-.-- == .:--: .      ..      :.         -::-===-   .     @%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###%%%%%%%%%%%#%%%%%%%%%%#%%##%%%%%#%%%%##%%%#%%%%%##%%######%######%############***###########*############%######****#########****###**##**###########**####**##*##%##********
// =.-..=- -++-::--:+*++**=.+=+::=: .:=::.      :          .-.:: :  :==-==-.=.-  ::.                      :  :  .    ..       . -#===-. ::--.  .    .:.   ...:        .       :         -%%%%#%%%%%%%%%%%%%%%%%%#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##%##%#%#%%%%#%%%#%#%#####%###%%#%%########%###*####*####*##########****#**##***######*#########********##**####*#*#*#********+########***********#*#*****#
//   -=:*--=:  ..==-+++==-:=-=--: : ::=...::           ::. := ..-   .::--::.-=:=:  :                       .  ::              :.   : @* :::-=: ..   :::.   :::             .             %%%%%%%%%%%%%%%%##%%%%%%%%%####%#%%%%%%%%%%%%##%%%%%%#%%%%%%%%#%%#%%%%%%%%###########%%#%%%%%####%#####%#*******#####%#*###*##############***********************##**#******#*#**#%#######******####%###*#**########*##*##
//   :=:..:-:  .-:=-=-::-:.-=+:-: .-:: ::  ..::...  .:  ..::: . . .-=::--.: ::---                       ... .                    .. :=:#=:.----:.    .::...  ::        .:            =:  %%%%%%%%%%%%%%%%%%%%%%%%%%%#%#%%%%%%%%%%%%%%%%%%%%%#*#%%#%%%%*##%##%##%%%#%%%#%%##%#%%%#%%%%####%%#%########%###**#####%######%#######*#####%#########*###################**###########*##**##*#########***#*#*##******###
// "=-:. .::-.-=::*===-:.:.-.:...  :     .. .  :    :=.  :     :.--::::::::.  ::. ::      ::        .                .              :.@@@@@@+=-:-:: ...:      .       :.             :.  %%%%%%%%%%%%%%*##%%%#%%%%%%%%%#%#%%%%%%###%#%%#%%%%%%%#%%#%%%%%%%%######*#+*******##%%####%######*##*#######%%#############*****#********#**##*#######**===******######%##########*##########*###############*###****##***
// +. .  ---::==::==-=+*::   :     .:. -:   ..    ..:-... :      .. .:.::-=:-=-: : .     ..                          .                 @#@@%@@#+%::    :              :              ..  *%%%%%%%#%%%%%%%%%%%##%%%%%%%%%%%%%%%%%%%%####%#%%%##%%%%%##%%%%###%###%###**###******#*##****#*########%#####****##*#####************#***#*#****#*#######*##*********######*#*#####*#####*#*#############****#******#####
// ..=: .  ...:=*=.::-*- .   .    .== :.  :.    :...   ::.     :.   :. ::::=--:...:::.:     :    .:                  .               :=@*%@%@@@@@=:                    . .           .   :%%%%%%%%%%%%%%%%%%%%%%###%%%%#%%%%%##%%%%%#**###%####*##%%%%#%########*#######*****#*##***#**######***###########*#***#**#*******####*########****##**##*#####**********#****###%#*#####*#########**########*#*#####*#*#*
// =:: :=: .: :=-...::::...    .:.=:....:                 .   .:   =:::.::==. +::::::             :.             .:    :==           .-%@+%%%%@*@@:.                                 .    %%%%%%%%%%%%%%#%%%%%%*#%%%%#%#%%##%%%%%##***##########**#%##%#%%#######*####*++++++**#%%#*##**###############*#########**#####***##*##***#*#########%###****##**###**#*#***#*********#############%##**###########***###*
// "..=.  :   :::=.:::.       ..:   .    .                  . :  :-:  .:.::-:-=-:==. :=:   ...   . -          .    :-.  :*.           --@@#%*#%@@%-                       .             ..=%%#%@%%%%%%%%%%%#%%%%%%%##%%%%%%%#%###*######%%##########%#######*###*#######*#####*####%%%%####**####%###########%###*######***#**###############********###########*###%##*########*#***#############*##*#*##*********
// %-*:=:::*==. : :.::  -.  =    .   .  :                 ..  :  -      ...:-::.:=::=:            .::                 :....          = .@@#****#*@==                                    :. *%#%%%%%%%#*##%%%%%%%%%%%%#*#%%%%*%*#%%#%##########*###%%%%%%#%%%%%%%%%#%%####*###%%##*####%##%%#%%%#####%###%%#%%###########*#**#****##############*####******###****#######****#####*******#########*####**####*###**#
// ". :-=:= :-::*::=-:.:     :   . .:   ....  .   ::       -:  :    .:     .: .::=-:-=:         .. :.              ..::++%           : :.:*%#**%@%%:::          .                          %%#*#%%%%%%%%%%%%%%%%%###%###%%%%%%%%%%%%%%%#%###%###%%%%%#%%%%%####%#%########**###%##*###%###%##%#*#***+***##*##########*##*##%####**#########*########**#*#*****#************#***########*******###**######**#*******
// #*%@%@=::**:=+*.=*+:  +. :           .::..:::..=         .   .   ..  . ....:=*::::      ::      :.                  .+ :     .     .-:.-=%==@@**%#=    .                                @%%%%%%%%%%%%%%#%%%%%%%%%%%%#%##*##%%%##%%%***#*#%###%##%#%#%%%%%%%#%%#*####%%%#%#%######%%#####*##%##%#**#####***####*#*******######*##***#****#####*#**####***#######*#**+******#*%########*#####***#********#**##****
// #-++::%--=+*==:-:-:= #%.-     .-   :.:=-:=::=:-::--:=:...   .   :  ...:  :  ::::- ..   .: .      .=: :         . . .               ..:--##%*%#%@%#*                               -::   @%%%%%%%%%%%%%%%#%###%%%##%%%%%##%%####%%%##%%%%%%%%%%*%%%%%%###%%%%#%#####*########%##***#**######*###**############***********#####********+**************####***########********#***#*#*******####**######**#********
// %%*:=*=:.-===+--.**:=:=: .          :.=#=.-  -=:::::-*..  .         . .     .::::.          .  :.: .  ..      .     =  :        .#=...: .#%++##%@%*    .                .          :.   -%%%%%%%%%%%%%%%%%%%%###%#*%%%%%%%%%%%%%%%%%%##%#*###%%%###%%##%%**#####%####%%%%%##%*%***##%#*#**#**##*+******###**#******+*+***+======*****###********#*++***#*#*****#*######*####*#****************#*****###*###*==*#
// ==*=**:==-.:..+-.*-=-*:.        .  #+. ::=.----::   +::          :    :     . . :...       .. ..  .                 -  :        .    . :...+@%**%%@=.          .         :          .   .@%%%%%%%%%##%%%%%%%%%%%%%%%#%%#%%#%%%%%%#%%%#%%##%%%%%#%%####%%###%%*##*#*********###############*##************************#**##***####**#******+**++++++******##*******************+***####**###**#*#####*##*#######*
// :-+==:#*-:   ==-::+=+==-  : = .. :-:-     . :  .=--:=:      :   .    .         . . .         .     ..             -=:                   .:.+:=%@@@%*:                               - .  @%%%%%%%%%%%%%%%%#%%%%%%%%%%%%%%#*###%%%%#*%%%%%%%%%%###%%%%%%**#%###*######*######%%####*#*###***##**#*##*###%##***#####%###****##*#*+**#*********######*##**#*###****+**##*##****#*****##*****##*#*#***#***=********=
// ++=-=+:=-=:=:-==**+=:==-==@+  ...=::+      ::  .. .=-:..::   . :                ..:.         :         .      .::   :                   :   :+=%#*#*-.                                :  %%####*%%##%%%%%#%##%%%#%#%%%*#%%%%%%%%%%%##%*#**#####%#####*##%####*#######**##**################**#*=++=+***********######*#*********##########********##*****#*****++++**####*####*######*****************###*******
// =-**+++%#*=-**==+* :+--+-+:=--%+               .   :.-::.: =- :.  .  .            .                 .::              .                  :     :*:--*+.                                   -#%%#%%%#%%++#=+%%%*#%%%%%%%%#%%**+*##%%#%%%##%%%%%#%##%#####%%%%%##%%%#%##*+*#%##%#*###%#*%##*##******%%###**##%####**##****************#####%****##*****##***###*******#******+**+*****##**##**=***#******##*##**##**
// #:===+*#+=*=++-:-*=#+**==:=::..+    = .          .==. . :--::: .-:..:.     :                        .  .             .     ...             .  ::-:::=*:.                                 %#%%%###%%%%%%%%%####%####%%*%%%%%%%###%#%%######*#**%%%#%%#*+**##**%###%%%##*******##****#####*#**########**#******##*#*####***##**********************#*+++*****++**##**###**####***####*************#*#*************
// ::=::=::=:*++=+#++*****+++:--:==:  .:    . : :  . -  :.   ..= :  ::: : ..-:::::       .    .                                .:.   :   . .     . ..**#@%::                .               %#%%%%%%#%%#=%%#*#**#%##%%%%%%**#%%%%#**#%%%##%%%%*#%%#%#%%#%****+#%#%#*##%##***==#%%######*******#******##*##*******#*########****************************+*********+***######****##**=+****##****+***##**=****+*++***
// ""-:=*=--:=.*=.=:-==*--*===== :-   ..       :    -..      .=:  :%:- .:.. ::::::                  :    .               .     ..:.  .       ..    :   +%%@+-                               ##%%%%#%%##%%%%###%%####%#******####%%%%#%##**##*#####*#%#######****########*********#****####%%#***#******###*#**************##*******************#****************************#*******************####******++=+*****
// =-+=*=**#=:-+ :=#:==::*===*:*=+:  .::    :   :.  .       .-: .-%  .:..:.=..:::        :        .:                   : =      .:. .:      ..    :. :  -:-@+:                         .    =%%#*+=#%%#+**%###*%#**##**##%%%%##*##%*###*###%#%#*=+##***#*%#####***#*##%#**##%%%%#*###***#*+**********#**##%%####*****##***#*#%#****#+*****==++****--=****#*#********++=+=*********+**#*##*##*###*******#***********
// +*=++*+=*=#*=:==:-==.:::-*#+*%+:  ::     .   :  =        :.- :=      :-..:- ::.:                     .                     ...   ..      ....:. .        %*:.                          :. %%%%*####%%*#%##*%%####*+##*##*###*#**#%%#*%#%##*####%%##***+=*****#%######%#*###**#*+**#######**#****************************##**#*******************+**********************#*+**+***************************#####***
// ++=++*=*#==+*=++:=*+=::=*-:=**@#%*@*:: :  .  .::-.  ::::-=::=-:       . .  :-=+=-:                 .                              :          : =-.       #+*:                            :%%%%###%##%%#####%%#%%##%####%*%%###%%##%%#%###+#%%#*#=####%##*##%#%#*#**#****####%*##*****##******+**#*##*****+-**#*+*****#++*+++**+***######**+****#**#****##********=*+++*##**#*#***********++++*+******+***#******
// #+*+*+*#*+***+++*==+-:**+*==*+=***+:=.-::.:   =*:=.: :  : :-:              .::::::         .    .:                                .. :. . .    .:-:.   *- #+:.                            %%%%##*#%*#%%#%##%#%%#%##****###**#####****+**##***%###%%###%%##%#*****##***#%#*#*#***+***+****##****************+++******#*****************************==++********++****************######****#*********************
// #******+===***=+*:**+=*==+++=+*++*=+=*. : =  :=#:    :                    ..-:::-.:.       .  ..:.:                  .              .     ..     .--    =  +-..                           #%*%%*#%%%%#%#%#%%%#%##*#*%%%%%#%#**###%%****##**##%#*******#=+*#%##***%********##%%###**##******###******#*##***#*****=*++****+*++***#***#*#*#******++************#********#*********++*******#******#*****##********
// +==+**+%+*+====**==*#%-====@@@@=++=*==-::.  :=-::     :.:.                    .::-::     ...:    :::-.             .                       . .   ::.    =:  *.                            #***#%#+++#%#*#***%####%%%**#+##*#%#**####*##**#***#%#%%#%*##******##*#*#***=**#***#****#***###***********+++++*****++*********************************+******+************+***********++++*********************+=****
// -=*+=+%#*++%***#%**=%%*##**+*#=:#*======:+ ====.    .: :         :           :  :::       .:: ..   :::            :                        .    .::      :*.%@=                           %#%%%#**##%%%#%%%###*##*%#%%#%#++*####%##+*%#*****##*#**###*##*******++**##**#*##****##*#**+=+=**++******#********#***********#***+++*+***#++*+++*******#********++**+*************+*****************#*#******+*******
// ""====+=:++*+****+%#*-+##=**+*=+==:-+=:=:=:.--   .  .  -         .                 :    . ..     . .:                              .              .-      ::**:                           =*%*+*#*#*#*******%#*####*###*###****#*#*###***#%#*#*##***###*#***+*#**=+*+*#*##******++**********#*+**********+**********++==++=+******++=++++****++===+*******++*****#*******************+*++++++===+=++++**********
// +::%.=:--=**:===++@#@@*==-==++**=+=*%=-=+*- *+     @.:#.          .                .:....::        .  -                                           . :      *#*=:                          =*%#%%##+**###%%%*####**#***#**#**#*##*%##*****+****+*##%#*#**#%#*#*###******+***#+#+*#***##**+=*##**++***#*###*++==+*++=*==*#**++******+************#*************++***************+++=++***#************#*#*********
// -.=*=#:-+=-++=====*+*@*#*==+=+*+==#=+*++=++:*:.  #:+:=: =:        .          .    . .:            .                 :                 .          ..  =:    .@#-:.                         =*%%#+*##%%%%##%%##*==**+=#*#*#%%##**+==*#**#%%%%%%%#=+*****#%#####*****=+**+****#*******#************#****=+=++***#******#*******++******==========+****************+**=+=+***+****##*********+***+*++*+++*+++++++++*
// =-*#+=+%=*=+++*+**+=#+=*==*-++*++=**#**+#*==%=+*=:*:-=:*:         :                ....   ..:-    .                                              ::.:..     .#=:.                          =**%%*###*+##*%****%%%%###**####*%%%##*###*####****#*=**#*#***+=+**+***+##********+**+*+++*****+==+*+++=*****+******#***++*+=====+++****************************++*+*+++*************++=+++***************++******+++
// =+==**@--==:--===+%%@*-==**++#*++*+*+==*=+*=%=+:-+*-.:+- .                ::--:...  :   .    .        .                                        .  : .::.     %*-..                         %#*%%%%##*#*%##%%####*##**###%%#####**#*####**##***#***#***+*****#*++*******+**#*********+****+*#****+*****+**+**+=***+=++**************+==*+++*+*++=+*++=++=====+====++**+***++**+**+**+*++***++++++*+*****++*****++
// -*-*+=.:-::-:*::*+=-:+--=+=-==-==++=*+*##***#==:*+-#+:-=.-##+ .   :.   .    :=   -:.-        ..                         .                      .. . ..:       %+=.                         =*****%%##***+##**%###%#%#%##+*%%%#%%%%*#%%#%%#%##***##*****+*****#****=*****+==+*+************==+**************++===+*****+*++***++*==*++======******+**+++****************+**++**++**++***+*+++**+++++++*****++=+*+
// +*#-:=.:-* .**==+:=:-+=:=-..:==:==:===*+::*#*===:=+#*=***:       ::           :::....:             :                              .           . : .   :       %*=:.                        %####***%%#***#*##*#%%*****#*%#*%%%#%%%#****%*+++***##+=**+####*#*##*==**+******=******+=++******++*=++=***++**+********+*+=+****+=+*++==+*****+***=**+++====*+**+====++*+=+*****++***+++++=*+===*+===+***+**********
// .--::-:::+--=:==+=:.*.*+-**+=-#-:=.::==::::::+::=-*#++#+=:::*.: =              .:..                                         ..     ..        .::..  . :       %%*:.                        *##**#*###%%**+##*+*##**##***##+*+=*=+**%#*#*****+*******##**#######*++==++++***+**+*#**++*++=++****+=*++=++=+====++=+***++*+====**+*****++==+++++==***+**++++****++**+===+++=+++***++++++**+****++=++++++++++=++++++
// = ::=-=:=--:*=%#**##****+****++=:...+-.:..::::::=**-++:+ ::::= -=.    =+                :-        .  :                  . :.... ...     .   .-:.:   .  :      %#*=:.                       =*%%%%#%%#@***#*+*#%%#%%%##*****%*#***==**=***+==****+*+=*+*****#%***********+*##+++*+*===+++*+==+*++*=**++*++=++++++*+++**++**+====+****++*******+*+==*+==+**+*******++++*====++++**+=+*=-=++***-=+*+*=+*+=*++=***+*
// ::*=:--::=:-:=-=+**%-++**===-:+-.-.==... .::. :*==*-++:  +  :.=:       .                ..                              ....      .        .-       ::        #**=-.                       @@@%%@%%%%%*#+**#*****=%%*##**#***+#**#%###*##%#**+*=*+=****++++*++***%#**##*+===****+*=+*=+++*********+*****+*+****++++++++**++********+**+======+++++++*****++*++=++*************++===++*+*+**++***+*****+==++++**+
// ::=:::::=-.-+=-::=*- -*+=+**:=:.:. -::. ..::  .:#+=+***         .:::--    ..  .     :                .               #        .:              .  .-=*.       .#%+=-:.                        .#%#*%#*##+****#*=%+==***@+=**###**#***=*%#*+++****+++++****#**##*#**+=+**+=====*+***#***++****=*+==+***+*****+*********=****+=----===+=+******+++++==*===+==+++====+++++*+++*+++=++*+=++*****+*+*+=++=+*++*++*++++
// =-: :=.:::===--=++--.+=*==-====:  .-...  . . .::::::+==       .-:::::.:             :..:                                       .:              .  .::: ..    .#%%=-:.                        ##%#%%*+**#*#****+=**=#*+*+=+#++++==**+**##%***+===**##+**++***+*+*#************++==****+***-====+===+**==+*****+====+=++==+*****==++==+++++++++++=========+++++=+**+*++++=++=+===++++**=++*========+++=====+=+**+=
// =#+--#::===:=:.:===- *+:= :.-.: = :       ..   :*:= +:.-  . : .    .+:.-       .       .            .                                      :       -::       =#%%%+-:.                      *@%%%%@%%%*#%*####*+=*==*=+-*+*++##+++**##:-++*+=*#%#*++****+++***++*=====*+=***=*+=+==++-++**+**++********+-+=++**+=++***+===+*++++****+=**++===++==+=++++*++++**==*+*++++*****++===-=+=++**++*+*++**+*====++*****+
// -===*-:*===+-==--=+:::=*:=:.:.=+::  . .    ..:: .::     ::  . : :-==+*+++-*                                                                    .   ::        *#%%*+=-.                      @@%%@%%*#%#%***#*+****+%*#*+**+***=*+**+++****##**+++=+*#**+==+++**+++*+**=--==**+****+==**+**+**=*****+=++*#**=+**+++=++=+++=++=+++==*+*+==++*++++==++++*++++*++==+*+++++=========+=+++++++**==+++==++++=+*+++++++*
// -===*:=+%%*+-==-:+**=+==:*.:-+=..  :-     ... :     ..    . :-=+:*=*+ -+:.- :                    :                                               :. .   :   :**##**+=:.                     %**#%%**+*++******+==+***#=%=#%%##*=***#*#***#****##*++**+=*%+***+*++*=+===+**#*+***+***+===**=*+*+*+++*=+-==+=+**====++*=+*==+*****+::==-==++-++++**++=++==+++=---====-==++*++++*++=++=+=+=====+==+++==+====++++===
// :=====***=*+%=+****+-%==+*:==*===:-:-=..-:  :.::.:-    -  .    : :-:*:: ::   #                        = :            = :                        ..      ..  -*#*****=-.                     @@%#%%%%%%#%@%@###*******=+**+**=+=*+++*+++******##+-=*#%####***#**+***++**+*==+=+===+*****==*+====+==+=****=+++++=======+***+++=========--=++***+*+++++========-====++===+++=++=+***+++++**+==++=========++++===+++
// :+:==%:=+-#+==%+#*:+-=-*%:=*-:=-=::-:- =+=::=:. ::--:     .      *.=*#*=%   = +                      : ::.                                                  :+####***=.                     @%%@@@@@@@@**+*#*+=%%***%*@*==+=+++*+#==**#**+*%-==+###=*+*###%******+**+++===*****++=:=====++++=*++****===++***=+******#*==-====++=+++*+**+=+==-=-===+==-=======*+++*++*+===---===+=====--=====+==+++++===*++==+=+*
// :==*%==#=*@%*#-+==-*:**=%%+=:=::-: :-:::-:-::::::::.-  :    -=-.=+=+=:=*= -# *=-.                   :  :-.             .                             .  .   ++##%%%%*=:                     @@@@%@@##%%%@%*+**#*##%*=#%%#======****+**=--===++***=+*##*****=+++===++*+***+=+*====+++**+*+*++*+=========+++*++==-=====+++*++=====++++==========+++++===+++**+++=====+=+++==--====-:-===+*+++====+++====+++=======
// -:@===:==+*==%=*-::=.+:=*-*=--: ==:.:::::::::=::::::=-*-      .==*:=-==-=- = : . +        * =     :                                            .    ..  . : *+*##%%#*-..                     +#****=*=##%**%%*#%**+***+-++#+**#++**=-==+====*##**==***+====****##+=:==+===**++*+***====*=:=======*++*==:=======+++++=+=+==++**+===+=-+*=+++=====+++*+*==:-::=-:::=====+===+=-+==+++++=====++++++*=+*+=++==+=++++
// #%*.-***@%%======-..--:::=+*====: .:.:.::::::..:::..:::::     ::= :+:#*====:-::.:::.: ::   =-:.::-.            =                                 :   .      *+**##%#*=:.                   +#@@@@%%@@%%%#*%##*%#+*%****##%%%*#=*+*=-==***+**+++*#*#++++**+=+++====+*++**+=+*+**+===-=+==*+=++++***========+++*++++==+====++++=+======+++*+++=+===========++====++=+++++*+++=++++==+++++==+======================
// ""%##=.=+*+*#*-@%:.=*--::.-:+=: ...::. ::: .::.:..:..::=:=  :=   --.*==:=.=:::.:= : +=:- =:.= :..             .                :. :                        .****##%#*=:.                        :@@%@%@@@@%@@%%%%%***%@@%@@%=-+**++***+*+****+**++**%*===+++++*+*==*++*+==+--=***==========:**+++=+=+==++*=++**++=+*==+===========*++=-==-=====+++++======+====+++++=+====+**==+++==+==++**+==========+*+++++++=
// =+-==+=#.:+:--*%==.=-=:::.-:::: ...::....  ::.-....::::::-:.+: .+--:=.==--     ::=  :+.. -===::+-        .   ===  .               . .                      :+**##%#*+=-:                           :+@@  :+@%%%%%%*+#@%%@@@%@*+*===*-+#%*#++=+**+#*+=+***+****+=+=+++*+**===*+==-==++***+++++====+===*+*+=+++*=++==--====-=====+==+========+======+=+====+==========--==============+**+*+++========-=-===-====+
// =..==-:*.- :=-:..:-: =.- ..:.:-.=:.-::::: .:.::::...... ::-=.   :: -: :: .=*== ::::*   : :: =*+=  :      :    .: :.:                       :  .:        .-:=+***#%*++=-:                    :-  *.+@@@+     +#%%%%%%%##%@*+%#*%@%#%#+=+==+-=*++**++##++=++:::=+**++**+=====+*+*+===+++*+=:--+===+++**=======-===============+===---=======:-=+++++=+===+++===+:==========+=++++=+*+===-:::=============+=+=+====
// ": .-.:+- =.:=..:: ..+: :...::. ....:.....::::..  ... .=.::: :. : :=. ==*.  ::=  :- ::   - ::.: :        :.   =#                              ...         ===+*****++=-:                     ..  :-@@@@@      :%%@@%@%%@%%%%@@@*=+=-::==#*===+++**-====--=-==+====+****#*===+==++======+=+=====+*+=====-==========--=============+====+=++++=============-=======================-=-===+=+++==++===+============
// = :*=.%===  ::.:    :   :..    :..=..  :::+:. ...:::.  .:::==    :=*::=:.  : +:  :  . - .  =..::      .  =          -                  .                 .:===******++-:.                    -%:=  **%@%%%@@%=*%%%%%@@#%@%%**+=+=+-=:=***++=-=+==:-::-+===*****+==++*+===****==++====**====+=+=--====-==++========++==++=-========++=++=============-=======++++=========+++==+=========---=====++==+========+=-
// ==-=*::--:= =-=    =: :    .     ..   :.:..: ...::. .::.=-::-= :.:::-====.-. :- .  :. .   .           : .:       %                                      .::===+*#*%#*+=::                   -:%@@@%@@@@@@@@@%@%%%##***#**#%#*+======+*+====+===:====*++==+*++=====*+==:-=-=*+**+**===============+-==++=++============++=====++=+========+++====---=========----===========-======+=============================
// - : .%+=-:+=-.  -==.. .           .  ..-  :....:..  .: =:.   :: ::=:=-:=:-. :=.-=-       -     .   :-  ::-                        :                     :.:====+***#**==.                   :@+####%*-.== .%@@@%@%%*=+=**#*#+++===+==**-=*====%*=*-=+*==:-=*===+==:-+***-=+=+*******+--===+=+*========+*+---===+=+==+=+==+===-====+==-============+======+++++====++========-====+=-======-=====+=====+=+=======
// %*=. : =-:.=:=  .: .  .      .   :    ..  .  ..   .....   :=   .. .:::::: =  =: :=-:    :  .  .   -::...-     .                         .             .  .:====+******+=:                   =@@@%-  .=@%%%@#%@#*===+=#*+===+*#**+=***=-==+*+=+====++*=-====+=-+=-=*+-==+=++==+*++=+=++++====++==+++========++*===-==============-=========--==--=============++==-========-=-:-===============----::--:-========
// =+-*=--   ::=   .  .   ...  :  . ..    .        .....::.         . .::-:+==+=---=+=::. :  .  : .:. .:.: :-                                   .             ======*##***=:                       :..*##%#**+%#++*+#%%%%*#@#%#+***%-==-*+====+*+==**+=-==**====::+*+--:=-=+++*=====+*+=+****-========++=--========+=+*==+:=========---==========+===++====+==-::====-========+=======+==============+======+==-===
// +:*==-::    =    .     .....   . :..  .      ..  .::.  .. : :..:-*=:=:::.*-=:=++=+-:.+-:.=:  . .       .                                                   ======*#%##*+:                        = -=%*=%#%*+*%%#*+**%**+*===*%@##++*==-=*++**===-++*==+==-=-=**:===++=**+==*==-==:-===========+*=++===---======--=====--::-=========+===+*=======--====--=-:::-:====--=======-===-=========-===================
// :===: :+         :.    . ..   ... :  ..     .   :....:.  .  .=.+=:*#-:--:=:=+==:*+*=- .: : -:::     ..=.        .+                        :            :  :======+=*#%%+:.                          ::*+**%%*+*@%*=*##**%@%#***=+=====*+-===--==**-=====++-+**-:==**+++**=======++===:===========-::--=--=---:-====--=++==========+========-==============-=====--::-::--====+++==---===========-======++=======
// .*%:= *      . :             .    .       ..           ..: .:-+---==::::-.+:#::-=:.=*+=-=.:=  :=. -- =++=     :*                      .   .              :========*#*#*+:.                  :       .  .=*+%%***++.      %#*+#==**###+====+#*+=-=++=+--===+*==++====+========-==:--==*#*++++::==:::======+======-=====+===+==+================---:---:::-=========-:=--===-:===--=-=-=================-===::=:::
//    +%=   .    .      ..     .        .     ..  .:  .=   ..: -   : .::--:.==+==:========.====+-=+=+ =. . :::.=:=.      .                                  =========+****==:                         .: ::%%@@++===+=-.#+=**=***#+=+*##@#=++++**+*===-=*=***++---=+*-:=++=--=-==---==++=--====+*===========+=====-==+=:========+=======:--======-=--====+====--===+++=---=============+===========-==::-====--====
//    =:-       .                    . :.   ......       :::.:-=:: -++=::.:::=+=-==--=:=: -:-:*+*=*=+=: .:=+ : -:   .: .:-              .                  .=====-==-=****+=:                  .:   =.   . ***#%#%%%%*=:=@##@%%%*=+%@%=##+=+*+++**=::=++=**-======+=+===:-=:========-=-==============--=====-=--=============-===-=--:-:==-======+==========--====---===+=--======+=-====---====--:-======::==-====
//   =   = :.     .    .      *.  :          .    .. . .::.-:.  .:+=--*=+=:===:-:.=+--.  -:::=*=+:+*+==:.*::..:-:.  :.=                                     =========+*#***=:                              =@@@%*#%%*==#*%@%%+=**%%#*+*%#%%##***++*****+===+=-===*=::==-=*+==::=---:===:.::::=======+=:::-=-:--=======++======::--:::--======---=:::--=---=---=========-======++====--:--====--==================+=
//  =:     ..    ..         . -              :.           . .: : .:.::   - : ::.   ==    :. +=*:===*==- :==::.:::.:.-.:                                     ========+*#%***=:.                   -:.       *@@-  :#++-*%@%*%*=-:.  =%@###*#*=*=:=+==*:-=+====+=-::--======.::::===+=::======--======-=====---===========+=:-:-======+=-=========-=======-=====:-===========--=:-=-=====-=-===========-========-::--
// : :                      .         .    ..                 :::-==   ..  ::: . ..   - -. ..::-==-==-==*-:+--=-::::.::.                                    =====+=*##*****=:.               ..::=%= .*#+ :+#@%@@%:::.. :*#%*===**=:--  :::===::-:---===+==*=-====-+===+=::-==+=::--====+=======--:-::==---=+*====:::-=--:====:---::::==============--==================-:-::--:-:---===--===++=-=-=======--=======
//                  .     :        ::.  ..        .   .:  ::    . ::..: . :. =.:=:-  ..-: .  :-:==:+=:+==::==**=-+: .  :       .                            .====+****+=+**=:.                     @   :=*+ @@@@@%=  ::.::==**===*+#* :-:*=-:========+++==++=:=:-=====+= :====:--=+=*+++====-:::-=:-=+=-=+=-:::::-====-::===--=-=--:-=-:-::=========-====----:-==::----:===========:-======================:==-==--
//    :                :.          .                .....        -=:.::.:   . :::  =:: :.   .:-=:--=:+-=:::+=-:====:                                    .   .+=====#*+=++**=::                     #   .**+#@#*%@%@@+    ..@*+**==**#*++#+%===**+=+*+=*-=====--=--::+=-:==+=====+=--==--::--:-====+**=-:::.:-=---======-::-:=------::::=--=-==-=====--====--===---=---:-::==-===--==========----================-==
// -.  = ..           * +   :                             .        ::..:.:-:: =.:=  =      .-..-=--=:*+*+* :-=::. -==.        .::                       .    *===-*+*=++##*=--.             .:.*.*%**   :*=#%#%**#+**+%. .@@@%@%%%%%#+==+*=+*==+**===+-=-===:::::=-=----=+===-===+=--======---++*=--==--========-----::---:::------==-=-:-==-====---=--=---:-==:=--=-=====-:======:::-::-=============-=====-::-=--
//   :=+:     .. .     -                                   .:.        ::::-==:-:    .=-=.:.:. =--:::==-+=::-:::-:::-*+-:-*+=====+*                            =+==*===#=**+==-.             ..: :     =     *@@@%#+=%=+**##*%%#@@@%%%***#*+#%*-=+++=+=++==:::-===---=-+====+*+=+=====-=====-===:-====-=====-==-:-:::::--=-=-=-:=======-=======-===-=-:::::-==--====-=========--:-==-=============-========-========
// =  .               :                                          .  :::.. ::::  :.   =  =:    :. -:=+*::=:= :.-::.:=:-%= + .. .: ::                           +%=*+==+*+***+==:                    *@        .:*%%++-:+%@**#%@%%%%#%#*%*=-*=++**+====-==-===-=:--==-===-::-=====*+++-:::-=-=====-=-==+==:-----==----::-=:--=-=--::--=--===::::-:::::::===-=-===-=-==--=-==::::::=--====-=======--:-:-=========-:===
//   :    :.  -    . =.                                          . ..:.===::===   =   ::  ::*-. :-:.*=-=. :=*-..:-**::=+ .::==:=::-* - .                      *%+#*==%###*%*=-:                       :         -%%*=-==%=- .:-++@%**%**@@-===+====::-:::=+***++=-----:::====-=:======-::::-=--:-==-:-:::--===-::::::---:=---====:::----------:::::=+=============----:-=================---=----=---::-----::---:-
// :.-:::.        . : .           .             .         .         ... -=:=-:-  .    :  -    .:    *:.= =#**-:=::::.. : :  -.:   -.::-.::=              ... - *****%#*#**%##=:    :             :=   . :      :%%*. .             -@*@@%*=*%%==+-::====:=*---=+===-======:---==+:==::----=--====:-===:-+*+.:::-:--=--=-===::-:::::-=-=-::::::::::-:::-====-::::::--=======--=::.-+=-:::--::--=:::--==---:::==---==
// -.-.:::-       =.     ..:   :                                     . :.:..-.     : .-            :=-+ .   :.=-::           .:=-:.    :-....       :=+  :.    +#+*%*#***%%%*=:.  :          ..%+ .-%*                                  **#*+*-=-:===-:==-.:======-==:---=======-:::=======---=::::-===+=--=::--==---=-==--:::::::::::::::--------=---:---==---=---==--====----::-----=====-===--=----===-==-----::
// +:          :.. #   :      :==                                   . ::..:::-        - .     : : .= =--    :-::  .+         .-       ++ =: .         :  ==    %==+######@%%#=:.#          .::.#  .:#-:                                 =++*=+=+=-====+=-::==::::=-==-====::-==:-======::::==--==-=====+=+++=-:::::-==--:::-::::-::::.::-::--:---::=:-::----==-=-=-==-==::==-:::==::::-:::======-:::==========---==
//    .    .....--:.        ..:::  :                                  ...:..:        ::       -    =  :.   .-...::          . . :-==*::*::               =*+* =+=-=+**##%%%##=-+           .::                                          ---:-=++:::==:::====:--::-==-==-:::-===-==::=--=-=-----::-==-=====+======-::-:::::::::::::-----:::::::======:::-===--:::::--:::.:::-:-:::--:---==------==-=-----===---=--=-
// = .    :::==+: -  :: :.  .+=. =                                 .     . :    :    - .      =   ..  :=   =*   : : =:- = .::::=+=-::=*.*=:==           %*+-=*-==+-+****%****%-:.            .                 ::       -: =            -:--:-=-::==---====:-:-=====::=:-=-====*==-=-:::--:::--::--:-:-=======-::::::::::--:::::----:::::---::::-=::--:===========:::::::-:--::-=---==--:---=-=----=:===-::==-::::-
// ..=   --::-=:.:   .:. : :=:=.   .                                    .-:.-  .              -: :=:.  -  -=  -::=-=: : = :::=-----.. .=:=:-:           =-   :.+===+=**#*#%#*=-:.:        . .::       #:      .:+*****==  :==:::. :.::.:=:====+-:=-.-=-=-=-:-==-==-:::::::-=****+++*--::--::--::----::=-:-==-==::::::::::--::-::::::::::::--:-:::::--:-==----:::::. ::----::-::--===----::--:-=--=-:::::::-::::::--
// =::..=.-  : -:    . . : ::.   -  :                                    ..  :.=.     ..   =* =- ::   :==.-:.:   .::::   -   .::..-*.==+::=-+-.     :  : :   = ++=-*=+*++###*+=+:         ...::  +::.    *. .-     #%@%=::  .-:...:::::-::::=---=-:=-==----:==-=:::::-::::=+**=-:---===:::-:-:::--=-::===+---:::::::--:::=:..::::::-:--==-====------:::::::...-=:-:-----::-=-=-=====:::::-::::-:-::--=---:::---::--
// =:..=.=  +-   .. -=    =-.::.     =                  -.              : .    :       ..       -.     = -:= : .::.::-::-* . :.... :..=.: ..=-=-     :  =   .  *+=*=+=*++**#%*=::           :::...-+- :=*=**       .=###+=-#+==-::::::::::--::-=::::-=-=::===:===:--:------=-:-:=-===--:-:::::::-::-==+=--=--==-:.:--::::.::::--:-::-::::--::::.::::::.::.:::::::-=====::--=-==-::::::-=-:--=-==::::-:------=---=--
// =.. .:  .:::- ..-:: *=:=+:- : := .*               .   :.    . ::=  .....  : :-             :.. -  .. :..: . : :  ::.. ::::..::*..  :.-  =.::.    - ==:  :   ++***++=*=**##==-:.          ..:=:::+##**=*=*@:      :-+%:*:=--=:==:::       ===-*=:::==-:=+=:::--:--:=::=--::==:::::::-::::::::-=::-==--=-:-==-====-:::::::-----::::--::-:.::::::::---:::::::-:-:::--:-------==-:--::=-=--::-=::-:--:----:::-=--:::
// #=  = -%-==-=:: :==*-.+:+::::. .       .    :      .   ..::. .:        =:-       ..      :    ..:     ::=: .-::.   ..:..::.   . + *:-+=.-   :   -  = .     :***%*%**==*#*++=--.        ..:.: ::: :+%#+=+*=-::::=*==-=-:====::=-::          .-===:::::--=::=-::=:-:--=---:-::::-::::::::::::--::-====----==-==--=-::::::-----:--:::::::::---:::::::::::---:::::::::-::-=-:-:::::::::.::.:::-===---:---::::::::-:-
// .=   .::-*--:...*-=:+=::  :.    .      :    =  =  :     .      :      .:=           . :-:   ::   ..        =: .       .: :-  -    ..*:: .* .: :=**:= :    .-=***%***-+*#=-=+==          . ::    :*%@*%@%%%#::-%+=:    -:--===*==::...         .::=*+-:::::=:--:-::-=--==-:-::::.::-::--:-::::-:---:::::-=-=====:::::::----:-::::-:::::--:.  ...::-=:::::::::::::::::::::.. ..::::::-::-::==------:-----=-----:::
// .+=: .:.*::.  +%:=:..:- :=.:   =    :-:   . :  :: .   :      - :  .    :  :-      :   :    .            .       .     =.:+.:-=: =   :. ..:+#+:* *===  .   - :*++#**%=*+===+*+=.       ... :: *#**+**##%##=-+*#%%*===:::-==**==*-==:::::::. . -+**+=:-===-:=--:=:::::::::---::::---::::::::.--::::-:-:-:-:-=--::::::--=-:::::::::-:::...::::::----::-:::::-=---:::::::.::::::::-:::::---=--::::-::::::-:-=-::-:::
// ::  ::.- *:.:%:::=     ::=+-==- =:  *:==.:: ..:---...:. ::   :            = : :  ::  :    .                      :  ::.=.  :. :: + +  : *#-.:-+=+**:  .  : *=++*=+##===***#*+=.       ::.. .  +==: .-+=:.-+=+%#=*++*+*=-*-=+----:::::::..:.::-==#+*+-:==--=---::::::::::::::::=-::::::-=----.:=:=::::::-::::::----:-:--:::::::::::.:::-==-----:::::::::::::::....::::-:::---::::::::::::::::------=::::-----:::-
// - : :  =-..-.:...-+   :::==.::=::=-.=..=:..:-  :...::  + ::::       :       ..  :   :.. :.               .     +   =:-:: . ..  .::= : ==#*#::%#**#@#+*= .::::**=+=**+++*****+=.    :  .:..     -=-::::::#=:--=====-===*#%%*=+**=:-:::.:--:::. .=-----=:::=---:=::::::-:::::-==--::.:--::::.::-::::::::::::::::::::-:-::::..:::::::::::-----::-:.:::::::::::::::::::::::::::::-::::::::::--::::::::::------::--:-
//  ::-::   -..+ ::-.=.-+..--.=.+.:: ..=.:=::=: :.    : :.- :         .   .        =::-=- ..=              :        =-::.  =   :     =:*+::.*-:**#*==-:+: :.+:-=+**++=*#*****#**=:   :    ::-       .+::.-:-=+--:::---:--:=+-=--:::::-:-::. :::::==-::::.::-::::::-==-:-::::::-:::::-==-::-::::::::::---=-::::::::::::::::.:::---::::::---=::::..:::::::::::::.::.::::-:-:----:-:::::::::::::-::--:::-:----::::::-:
// :-    :-= ..==-:-=:=..--:....=:-:.     :-:..:     :. : --:=- :      :.      .. :     =:: :     :      --. -=.-::=:.:.  :   .:  :: %=.  =**=**#*-** - =-+*.- :*#***+**#%#*****+: .=     .:.         .::.:--::====:::--:==-:--:::::::-:::--==-==::==       ::::--:::::::::--::::::::::::::::::.:::::::-:::::::=-:::::::.:::::-:::::::::::::::::::::---:::::::::::::--:::::::::::::::--:----::::::::::::::--=--::::
// ,=..+ :  =:=--:=== ::=#.: :   .-           .= .  ..: ::...   : -       :.      +    :     .          =-..   :=-==- :=+: :  :====#:*%#=+**#%==:==:-  -=-==+*+=.++****+****##*#+.:.        :       .    --+-+==.::::::::-:=:==:-:--::::==:-=--==.--          :-::::-::::::-::::::::::::::::::::::::-:::::::::::-:::::::::::-:...:::::::::::::-::::::::::::::::---::::...::::::::::::::-:::::::::::-:-----:.:::::::
//  =-::-  =*:-=-=:   .=:=:::. .        .    .:++.. ::: ...=:..  .: . -:    .:: :-=            :: .:. : .  .-:::-=.: ::   :-==.*=*++=:--%-*=-=+***%%= : =+-=::=:--==+***+****####+         ..            . :::--::::::::-.:::::::-::::-==------:::                ::::::-:::::::::::::-::::::.:--::-::::::::::::::::::..:=:  .::.:::::::::::::::..:::::..:::::::::.. .::::::-:--::-:::::-:-=-::-:-:::::::::-:::::--
// - :: : = -:-==+=+::=-::  :   .    :  . .     : :- ..    :  - :.: :       : .. :.         :   :.   ---:  -=-=-=: .   : *-%+#=-+++=::::-*    =*======.-. ::    -:==+***+****#%#+.         :                .-:::-::-==--=:::::--:::--=:::::---::.                 .::::::::.:::::::::::::::::::.:----=:::::::::::.:. ::-: :::.:::::-:::::::::::::...:::-:::::..   .::::::::::::::::::::::::-:::::::--:::-:..::::::
//  .-:==  .:=+.* :=.     :.  .::          :    ::=    +    :      :        -   ::.     ..::    :.         -=:=-::  --.*=:.::-*-.- =: =   ::=*=+=. +  :     : : .-=++***++**##**-                             :-----:::::::-=::=--:=:--::--::..:=                    :::::::::::::::::::::::::::-=::-:::::.::::::::..:::...::::::::::..::::.::::::::::::::::.:...::::::::::::-:::--::-:--:::::::--:..::::::::::::::
// =: :..:  =*=+: =.+=@@:..  :.:              .:  . ::   = :   .  .         .    :      ..:. +    :  .   . *:.-:-:++*==-+++*=*=  :=*.:::-:   ==-:   -.:-=-    =::::++=*+**%#+**#-.      :..                   .::-:::::-::-:::-:::-:::::-:-:::::::                   .:::::::::::::::::::::::::::::::::::::::::::::::::...::::::::::.::::.....:::::::::-::...::::::::::::::::::::::::::::::::::.::..:::::::::::::::
// .  :::. . +-=:== :    #  ..:             : .   :   :                                  ..       :    -. -. .:-=::.- **=: .=:=*%.-- :+#+    :  * =  *: ::  :.:.::***++****+*#*+-.==               -:-:..    ==::::::::-:::::-:..:-=::::::::::=-=:                    ::.:..:::::::::-:::::..:::::::::::-:::::::::.::::::::::::::::::::::::::::::::::::::::::::::::::::::-::-::::::::::...:::::..::::::---:---:-:::
// .. ==:.*+=:=*. .+-.      :-             . :  : .           *+:  .             .       .-:      ::=**  -: .-=-  :.    =**+#--=*:     =:= .   :: -  #:=.:=:.. :=::+*#****#%*%##=.                 --:::--:-=::::::-::-:::::::::==-::.::::=::::....                    .::::::.:::::::::::::::::::::::::::::::::.:::::::::::::::...::::::::::::::::::::..:::::::::::::::::...::.:::::::::::::::::::::::::::::--::::
// - +  ::+ =++.  .-=#== .::-    :         :        -+        *=. :-=:-       .     .::      = -::=:      :. -==:. ..:=***-=::.-=*+:. .  :   #      :  :- .=      :*+--==*#*#**+:.                 =::::::::::::-::-+=-:::-=:::=:::::=-.:..::                            :..:::---:::::::::::.::.::::..::::::::::::::::::::::::. .::::::.:........::.::---::--:::.:::::::..::.:::.::::::....:....::::::::::::::::::
// :-: -... *:-... :  .   .: .- .              :   .:             .      ::  .  .:=:  :.  .   . .:.: : .   ... .::.::-.=-+-::---=: +*:  -  =:*   :  . .:::. :=    =*+==-:+****++:.                 -=-:=-:::::::--+-=-::-:::::..::::.:-::-:--                            ::::::-:.:-::::::::::::::::....::::::::::::.::..::::::::::-:--==::.....:::::----===::::::::::::..::::::::::..::.......:::::::...:::.::::::
// =::: .  =- - ::  :    = : .- . .:   :      :   :               .  =  . .::    .=-      . .: - =.:  ..-    . .: ::.-. *=:= .::.:-*-+    +* -      == *-:--=====:+*-:::=-=****=:...               * .-=-::---=-=--====:::-::-::-=:::-:::-:..                             .::-::::::::.::::::.::::::::::::..:::::::::::#====:::===:::....::.:::::..=.:======*==#++:::....:::::..:..:::::::::::::::::::::::::::::.:.
// :.   :. +::.++   ..::  -: ::.::             :  -:          :   .==:  .  : . : .= . .=-:    ::=-.. .  .=  ..    *-=:::=*:.:--:-=:  :::=:+:*+ =   : :::-:-  :*-*=+==----==++##*==::               *:+*:.:==*===-====::::-:=:::::::.:::::::-.                             .:::.:::.:..::=----==-::::...::::::==-==::-::-::::-::=:.::::::::..::-====--:%--=:::--:::%. .::::::...::::::::..::::::::::::...:::::::::::
//   ::   #=::-= .::  :=-=   ..:-+ .:   :    .               . :            .:: : ::: -  =+::: . +- . : .:.++*.=-: ...:=:%:.:--::=======:- *%.-  + =   .  --:+=* :==-:::==+****=:.:.               ..++=#==*=:::=:::::.:::::=-::::::::::-:::.                            .:. ..:.::::::.::::--:::-:..:::: :::-=:::==::::::::-:-::::::::::::-:::::-:--=:.%%%%-:::::=:. .:.::..:::.::::.::.::::::::..  .:::::..::::::
// :   .:* .:-:  -*   ::::: : :.:::--::: ..     :               .    .   .=  .   :::=:=:: .=:    . .-::  :-    =**:..:..::-=:.. =*:-**.:::. :::=+- .   :::.:==-**=-=+-::-=++***=:.:.                       *.-*=. :::::-:::::::::::::::::-:-                             .::::.:::::::::::::--:::::::::.:::::-::::::.:::::::::.:-=::::::::..::::-=::::::-=::=:.:...-::::::::.:::::...::::::::::::::::::::.:..::::..
//      :  =::    .:.:==-*# .::::== -  ::     : :      ..:-=-                    =   :-:  .= :  :. =:: ..  =::   #= :: .=  .*=.::=#*#::=-+:=  :::  #  :.:-+=::::=.+:=:=+=-++*#*=::: .   .              :   ::@.  =.:-:.::::::...:-:::::::::.                           -..::::::=.:::::::::::-::::-:::::::::::::=..:::-=*=-*.::.:.=*::::::::.:=::::--*-+*%-::=:. ..::-=...........:.::....::::::.::.::::...::::::::
// ::--   :*#.    : :==--..  :: -=. .    -   :..- .                              -...:.. ::.    =. . -   -= =.:.::=.:--==: :: :=::::...*- : : :   -=:  ::=:+:: :::=...:::-+*#%#+:-:..    .            .           :=::---=     ..:::::::::..                          . ..:::.::.::.:..:::--:.              :   .::::+-=#***%%*::=.   ::--=+++==:=::==-=====-=---:=::::::..:::..::::::::....:::.:::..::..::::::::-:
//  :.-%. ##:+.  = :-  .   : ::..  -   : .:- : :=:=                     .  .     - : :::-..  =.:::.   =:-. :+::=:::   ==: = . :  :   :.. =: : -:+       -:::=::   . +#..-:=**#**=:==:                          .     .=:.:: :   .::::.:::::                           :::::-:=::::::::......                        =:#@*==%-:    .         -*#%#==::::==+=**:-:---::-==-::::..::.:..:.:::::::.......::::::::::::.:
// . -=-::=-:  :=.     ::    .  :.:: =  :::-.:...                               ::      . #:: ..:..-: :. .-+::-:.-:-++: ::.. .:  ..:.-+=*-   :..: .:. .   .  .:.-  -.. :===*****=:::.    :                                    . .:    .:..                            :::.::::..::..:.::::..                         @@*          ::      .::--::.. ......:::::--:::--====-:::::......::.... ......:::::::::...::..
// +=+:.:==  .:: :.    :  : -:      :    *:.-                       :           -         .  .  :.::..:%::= %+ .::+:   :.  -: :=    :   =-:       =: : .   :==. :  ::.  :==***##=-:.                                                   :..:                           .:::::. ..::.:.:-.:...                          =-                                    :::::--====----:::::::....::.....::::::::::::.:::::::::
// ::::..:* .:.-  .     :=:=: .    .:   *  :   .                                -       .  -    : ::. :==..:+ : -: . :    : . ..:=     =:-  . :     =*-*=:: +*=  . ::-=-====***+=-:.                                                      :                            :...::=:-::::::..:.::.                        -#                                      =-:::-===--:--=--::::::::.  .:::::::::::::::::::::::::
// -.....::+:...:  :== :=  = .   :::+ .: -*::     :                   .:       .     :      :  :.  :: :- :=**%++=+==:+ - . - ..:: -      .  :  :    .::* :.=-: .:  :=---====*+=+--:.                                                                                  ...:::-::::--===:::::::                       :::-  =                                  .::::---:::--:--:::::.::::::::::::::.::::::::::::.:...
//  ::-:::: ..:..:-=-:. = .     :        ::   :                  :             :      :          :.. -: .:.-:= -=*=:.:    %=.:.:  ::.: .=::. :.    :.-:..--=*::= :-+:=======*+===-:: .                                                                              ..::::-::=:--=:::::::::--:                       .:-.:.                                   ::::::::..::..::::::::::.. ..:::::::.:.::::::::..... 
// -:...:-  ::.:: ::. ::.   .:             .                                   +          - : :      .  .. +=. +:*:=  . :#:-==:  ::-  :  - :  . :=+ :=*-====+##*=:::=:--=====*++*=:::                                                                               .:::::-::::=::::::..:::::=..-                .....:::-:                                  =+-#+=.:..:::::....:::::......::::...::..:::::..::::::
//  ...: .: .  ....:-.    .    ..:     ..  :      =                                            -        . - -: .:=**-   -:+  =:     .    ::=-  ==: -=-==: --:*= :  : =***#*+**+++=-:.                                                                           ...:::::::....:::.=-::::::::.:. =:              ...::::-:                                 :::+*+-:::-::::.  ...:.:::........:..:::::::::::.. ..::::
// .:::   =  ..:::===          :. .   ::: -  .                       .  .     =  .              -       : : -:--. .:..:-  ==.:      :::#=:=::.-%=#* -#=-===*:= -: =:::--=+:*%=**+=-::       =                                                                   . ::::.:..::.....:=::::::---=-:.             :.:....::.                                   :=+=:::...   .    .:.::.. ....::...::.::::::. .   ......:
// =-..: %.   .:-=++      .. .   ..:       .                 ::               *       .        .          .:.:== . :.. ::  :..:.  :+=.   .:. : :::  :-==+=**  ::-:...:-===*-*#***+=--=                                   :                    . :. :   :- .- .:::-.:::::::..:::...=--:::=+::.::.             ::=-:=+*==*-     =-                      *-*+-+:::::  ...      ...     ...::..:..:::::.    ...........
//   .  %*:  .:-+=*-=-:      :::+: :             .:       .=:             =                    .          .::=-..:-:- . -      ::=. =-:::-=   .-= =:*=====+++==+=- --===+**++*=.**++#*     .  .                                            .: :::::=*@#. :+=.:-..-:....:::..:...::=::-:::*=:::...               :::::            +-                 .:=::::::::::.     . ..... . ...:..:..:.....=.    ... ..      .
// :=-:@==. :-:::::--=+..     .=   *: .     :           : .             .  .   =:   :.       +   .  :.    :.::=.::.--:-==     -:::.-:::::==:=:+-::-::===-+:===*===-:.:-:-+=+=:-=:=***-      :       .                                     :=-: -:=-::-==+-:=*+:-::::...::...::.:=+::-:::..:...   .. .          .::..:...:  ....   +#:              ..:::-::....:.   . .:::...:...:....... .:.   .  ...... . ..:.::.=:-=:=***-      :       .                                     :=-: -:=-::-==+-:=*+:-::::...::...::.:=+::-:::..:...   .. .          .::..:...:  ....   +#:              ..:::-::....:.   . .:::...:...:....... .:.   .  ...... . ..:.::.