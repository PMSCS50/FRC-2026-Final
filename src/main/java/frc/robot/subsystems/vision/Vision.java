package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {

    // *Create fields
    private final List<VisionIO> cameras;
    private final List<VisionIOInputsAutoLogged> cameraInputs = new ArrayList<>();
    private final CommandSwerveDrivetrain drivetrain;

    private final Debouncer alignDebouncer = new Debouncer(0.1, DebounceType.kBoth);

    private final HashMap<Integer, Pose2d> tagposes = new HashMap<>();
    private final HashMap<Integer, Double> tagambiguities = new HashMap<>();

    public Pose2d cachedHubPose = null;
    private Pose2d lastFusedPose = null;
    public boolean hasSeededPose = false;
    private boolean autoStarted = false;

    private Matrix<N3, N1> visionStdDevs = VecBuilder.fill(
        Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
    );

    // *Constructor
    public Vision(CommandSwerveDrivetrain drivetrain, List<VisionIO> cameras) {
        this.drivetrain = drivetrain;
        this.cameras = cameras;

        for (int i = 0; i < cameras.size(); i++) {
            cameraInputs.add(new VisionIOInputsAutoLogged());
        }
    }

    @Override
    public void periodic() {
        // *Get latest alliance
        refreshAllianceCache();

        // *Declare and initialize variables
        SwerveDriveState driveState = drivetrain.getState();
        Pose2d robotPose = driveState.Pose;
        double yawDeg = robotPose.getRotation().getDegrees();

        Transform2d robotInv = new Transform2d(
            robotPose.getTranslation().unaryMinus(),
            robotPose.getRotation().unaryMinus()
        );

        tagposes.clear();
        tagambiguities.clear();

        Pose2d[] goodPosesBuf = new Pose2d[cameraInputs.size()];
        double[] weightsBuf = new double[cameraInputs.size()];
        double[] camDistBuf = new double[cameraInputs.size()];

        int goodPoseCount = 0;
        double fusedTimestamp = 0.0;
        
        double globalClosestTagDist = Double.MAX_VALUE;

        if (DriverStation.isAutonomousEnabled()) {
            autoStarted = true;
        }

        // *Seed drivetrain pose at the beginning with vision
        if (!hasSeededPose && !autoStarted && DriverStation.isDisabled()) {
            Pose2d bestSeed = null;
            double bestScore = Double.NEGATIVE_INFINITY;

            for (int i = 0; i < cameras.size(); i++) {
                VisionIOInputsAutoLogged inputs = cameraInputs.get(i);

                if (!inputs.hasEstimatedPose) continue;
                if (inputs.estimatedPoseTimestamp == 0.0) continue;
                if (inputs.numTagsUsed < 2) continue;

                double age = Timer.getFPGATimestamp() - inputs.estimatedPoseTimestamp;
                if (age > 0.20) continue;

                double dist = camDistBuf[i];
                if (dist > 5.0) continue;

                double amb = 1.0 / Math.max(1, inputs.numTagsUsed);
                if (amb > 0.5) continue;

                double jump = robotPose.getTranslation().getDistance(inputs.estimatedPose.getTranslation());
                if (jump > 0.75) continue;

                // |Scoring: closer tags + more tags + lower ambiguity
                double score = (5.0 / dist) + inputs.numTagsUsed + (1.0 / amb);

                if (score > bestScore) {
                    bestScore = score;
                    bestSeed = inputs.estimatedPose;
                }
            }

            if (bestSeed != null) {
                drivetrain.resetPose(bestSeed);
                hasSeededPose = true;
                Logger.recordOutput("Vision/seedPose", bestSeed);
            }
        }

        // *Loop through each camera
        for (int i = 0; i < cameras.size(); i++) {
            VisionIO io = cameras.get(i);
            VisionIOInputsAutoLogged inputs = cameraInputs.get(i);

            // *Update IO
            if (io instanceof VisionIOReal realIO) {
                realIO.setRobotYaw(yawDeg);
            } else {
                ((VisionIOSim) io).updateSimPose(robotPose);
            }
            io.updateInputs(inputs);

            // *Tag processing
            double closestForCamera = Double.MAX_VALUE;

            if (inputs.hasTarget &&
                inputs.visibleTagIds != null &&
                inputs.visibleTagPoses != null) {

                Pose2d[] tagPoses = inputs.visibleTagPoses;
                int[] tagIds = inputs.visibleTagIds;

                for (int j = 0; j < tagIds.length; j++) {
                    int id = tagIds[j];
                    Pose2d tagFieldPose = tagPoses[j];

                    Pose2d tagRobotPose = tagFieldPose.plus(robotInv);
                    tagposes.put(id, tagRobotPose);

                    double ambiguity = 1.0 / Math.max(1, inputs.numTagsUsed);
                    tagambiguities.put(id, ambiguity);

                    double d = robotPose.getTranslation().getDistance(tagFieldPose.getTranslation());
                    if (d < closestForCamera) closestForCamera = d;
                    if (d < globalClosestTagDist) globalClosestTagDist = d;
                }
            }

            camDistBuf[i] = closestForCamera;

            // *Camera rejection/acceptance
            if (!inputs.hasEstimatedPose) continue;
            if (inputs.estimatedPoseTimestamp == 0.0) continue;
            if (inputs.numTagsUsed < 2) continue; // must use >=2 tags

            double age = Timer.getFPGATimestamp() - inputs.estimatedPoseTimestamp;
            if (age > 0.20) continue; // stricter age cutoff

            double jump = robotPose.getTranslation().getDistance(inputs.estimatedPose.getTranslation());
            if (jump > 0.75) continue; // strict jump rejection

            double amb = 1.0 / Math.max(1, inputs.numTagsUsed);
            if (amb > 0.5) continue; // strict ambiguity rejection

            double dist = camDistBuf[i];
            if (dist > 5.0) continue; // reject long-range solves entirely

            if (!isEstimateValid(inputs.estimatedPose, yawDeg, inputs.estimatedPoseTimestamp)) continue;

            goodPosesBuf[goodPoseCount] = inputs.estimatedPose;

            // *Camera weighing
            double tagFactor = inputs.numTagsUsed;         // linear tag count
            double distFactor = 1.0 / Math.pow(dist, 4);  // quartic falloff
            double ambFactor = 1.0 / Math.pow(Math.max(0.1, amb), 2); // squared ambiguity penalty

            double w = tagFactor * distFactor * ambFactor;
            weightsBuf[goodPoseCount] = w;

            fusedTimestamp = inputs.estimatedPoseTimestamp;
            goodPoseCount++;

            // |Log cameras here
            Logger.recordOutput("Vision/cam_" + inputs.name + "/estimatedPose", inputs.estimatedPose);
            Logger.recordOutput("Vision/cam_" + inputs.name + "/weight", w);
        }

        // *If all the estimated poses are bad then make EKF reject vision
        if (goodPoseCount == 0) {
            visionStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            return;
        }

        // *Dominant camera override
        double avgWeight = 0.0;
        for (int i = 0; i < goodPoseCount; i++) avgWeight += weightsBuf[i];
        avgWeight /= goodPoseCount;

        // |Fuse only best camera measurement if the other ones suck
        for (int i = 0; i < goodPoseCount; i++) {
            if (weightsBuf[i] > 20.0 * avgWeight) {
                Pose2d dominant = goodPosesBuf[i];
                visionStdDevs = VecBuilder.fill(1.0, 1.0, Double.MAX_VALUE);

                if (RobotBase.isReal()) {
                    drivetrain.addVisionMeasurement(
                        dominant,
                        Utils.fpgaToCurrentTime(fusedTimestamp),
                        visionStdDevs
                    );
                }

                // |Log here
                Logger.recordOutput("Vision/fusedPose", dominant);
                Logger.recordOutput("Vision/fusedStdDev", 1.0);
                Logger.recordOutput("Vision/fusedSpread", 0.0);
                return;
            }
        }

        // *Weighted fusion (created fusedPose)
        double x = 0.0, y = 0.0, theta = 0.0;
        double wSum = 0.0;

        for (int i = 0; i < goodPoseCount; i++) {
            Pose2d p = goodPosesBuf[i];
            double w = weightsBuf[i];

            x += w * p.getX();
            y += w * p.getY();
            theta += w * p.getRotation().getRadians();
            wSum += w;
        }

        Pose2d fusedPose = new Pose2d(
            x / wSum,
            y / wSum,
            new Rotation2d(theta / wSum)
        );

        lastFusedPose = fusedPose;

        // *Calculate camera disagreement
        double spread = 0.0;
        for (int i = 0; i < goodPoseCount; i++) {
            Pose2d p = goodPosesBuf[i];
            double dx = p.getX() - fusedPose.getX();
            double dy = p.getY() - fusedPose.getY();
            double d = Math.hypot(dx, dy);
            if (d > spread) spread = d;
        }

        // *Reject if cameras disagree too much
        if (spread > 0.25) {
            visionStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            Logger.recordOutput("Vision/fusedPose", fusedPose);
            Logger.recordOutput("Vision/fusedSpread", spread);
            return;
        }

        // *Calculate standard deviations
        double avgDist = Math.max(1.0, globalClosestTagDist);
        double effectiveTags = Math.max(1.0, wSum);

        double base = 0.06;
        double stdDev = base * (avgDist * avgDist) / effectiveTags + 0.35;

        stdDev *= (1.0 + 10.0 * spread * spread);
        stdDev = Math.max(stdDev, 1.0);

        visionStdDevs = VecBuilder.fill(stdDev, stdDev, Double.MAX_VALUE);

        // *Fuse fusedPose to odometry pose with EKF
        if (RobotBase.isReal()) {
            drivetrain.addVisionMeasurement(
                fusedPose,
                Utils.fpgaToCurrentTime(fusedTimestamp),
                visionStdDevs
            );
        }

        // |Log here
        Logger.recordOutput("Vision/fusedPose", fusedPose);
        Logger.recordOutput("Vision/fusedStdDev", stdDev);
        Logger.recordOutput("Vision/fusedSpread", spread);

    }


    // *Helpers
    private void refreshAllianceCache() {
        if (cachedHubPose != null) return;
        if (DriverStation.getAlliance().isEmpty()) return;

        cachedHubPose = VisionConstants.getHubPose(
            DriverStation.getAlliance().get()
        );
    }

    private Pose2d fusePoses(List<Pose2d> poses) {
        double x = 0.0, y = 0.0, theta = 0.0;

        for (Pose2d p : poses) {
            x += p.getX();
            y += p.getY();
            theta += p.getRotation().getRadians();
        }

        int n = poses.size();
        return new Pose2d(
            x / n,
            y / n,
            new Rotation2d(theta / n)
        );
    }

    private boolean isEstimateValid(Pose2d estimatedPose, double headingDeg, double timestampSeconds) {
        if (estimatedPose == null) return false;

        double age = Timer.getFPGATimestamp() - timestampSeconds;
        if (age > 0.25) return false;

        if (estimatedPose.getX() < 0 || estimatedPose.getX() > Constants.FIELD_MAX_X) return false;
        if (estimatedPose.getY() < 0 || estimatedPose.getY() > Constants.FIELD_MAX_Y) return false;

        double headingError = Math.abs(MathUtil.inputModulus(
            estimatedPose.getRotation().getDegrees() - headingDeg,
            -180, 180
        ));
        return headingError <= 90.0;
    }

    // *Getters
    // |For targets
    public boolean hasTarget(int id) {
        return tagposes.containsKey(id);
    }

    public boolean hasTargets() {
        for (var in : cameraInputs) {
            if (in.hasTarget) return true;
        }
        return false;
    }

    public int getBestTargetId() {
        int bestId = -1;
        double bestAmbiguity = Double.MAX_VALUE;

        for (var in : cameraInputs) {
            if (!in.hasTarget) continue;

            if (in.targetId != -1 && in.numTagsUsed > 0) {
                double amb = in.hasEstimatedPose && in.estimatedPoseTimestamp != 0.0
                    ? 1.0 / Math.max(1, in.numTagsUsed)
                    : Double.MAX_VALUE;
                if (amb < bestAmbiguity) {
                    bestAmbiguity = amb;
                    bestId = in.targetId;
                }
            }
        }

        return bestId;
    }

    // |For robot position
    public double getX(int id) {
        return hasTarget(id) ? tagposes.get(id).getX() : 0.0;
    }

    public double getY(int id) {
        return hasTarget(id) ? tagposes.get(id).getY() : 0.0;
    }

    public double getYawDeg(int id) {
        return hasTarget(id) ? tagposes.get(id).getRotation().getDegrees() : 0.0;
    }

    public double getYawRad(int id) {
        return hasTarget(id) ? tagposes.get(id).getRotation().getRadians() : 0.0;
    }

    public double getDistance(int id) {
        return Math.hypot(getX(id), getY(id));
    }

    public Pose2d getFusedEstimatedPose() {
        List<Pose2d> good = new ArrayList<>();

        for (var in : cameraInputs) {
            if (!in.hasEstimatedPose) continue;
            if (in.estimatedPoseTimestamp == 0.0) continue;
            if (in.numTagsUsed <= 0) continue;

            good.add(in.estimatedPose);
        }

        if (good.isEmpty()) return Pose2d.kZero;

        return fusePoses(good);
    }

    public Pose2d getFusedWeighedEstimatedPose() {
        return lastFusedPose;
    }

    // |Return hub pose
    public Pose2d getCachedHubPose() {
        return cachedHubPose;
    }

    // *External Methods
    // |Whether the robot is aligned to hub
    public boolean isAlignedToHub() {
        if (cachedHubPose == null) return false;

        Pose2d robotPose = drivetrain.getState().Pose;

        double angleToHub = Math.toDegrees(Math.atan2(
            cachedHubPose.getY() - robotPose.getY(),
            cachedHubPose.getX() - robotPose.getX()
        ));

        double yawError = MathUtil.inputModulus(
            angleToHub - robotPose.getRotation().getDegrees(),
            -180, 180
        );

        return alignDebouncer.calculate(
            Math.abs(yawError) <= VisionConstants.HUB_ALIGN_TOLERANCE_DEG
        );
    }

    // |Yaw to position
    public double getYawToPose(Pose2d targetPose) {
        if (targetPose == null) return 0.0;

        Pose2d robotPose = drivetrain.getState().Pose;

        double angleToTarget = Math.toDegrees(Math.atan2(
            targetPose.getY() - robotPose.getY(),
            targetPose.getX() - robotPose.getX()
        ));

        return MathUtil.inputModulus(
            angleToTarget - robotPose.getRotation().getDegrees(),
            -180, 180
        );
    }

    // |Distance to position
    public double getDistanceToPose(Pose2d targetPose) {
        if (targetPose == null) return 0.0;

        Pose2d robotPose = drivetrain.getState().Pose;
        return robotPose.getTranslation().getDistance(targetPose.getTranslation());
    }

    public double getBestDistanceToHub() {
        if (cachedHubPose == null) return -1.0;

        int hubTagId = VisionConstants.getMiddleTagId();

        if (hasTarget(hubTagId)) {
            return getDistance(hubTagId);
        }

        Pose2d robotPose = drivetrain.getState().Pose;
        return robotPose.getTranslation().getDistance(cachedHubPose.getTranslation());
    }
}
