package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {

    private final List<VisionIO> cameras;
    private final List<VisionIOInputsAutoLogged> cameraInputs = new ArrayList<>();
    private final CommandSwerveDrivetrain drivetrain;

    private final Debouncer alignDebouncer = new Debouncer(0.1, DebounceType.kBoth);

    private final HashMap<Integer, Pose2d> tagposes = new HashMap<>();
    private final HashMap<Integer, Double> tagambiguities = new HashMap<>();

    public Pose2d cachedHubPose = null;
    public boolean hasSeededPose = false;

    private Matrix<N3, N1> visionStdDevs = VecBuilder.fill(
        Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
    );

    public Vision(CommandSwerveDrivetrain drivetrain, List<VisionIO> cameras) {
        this.drivetrain = drivetrain;
        this.cameras = cameras;

        for (int i = 0; i < cameras.size(); i++) {
            cameraInputs.add(new VisionIOInputsAutoLogged());
        }
    }

    @Override
    public void periodic() {
        refreshAllianceCache();

        var driveState = drivetrain.getState();
        double yawDeg = driveState.Pose.getRotation().getDegrees();

        // Seed pose once from any camera that has a good estimate
        for (var inputs : cameraInputs) {
            if (!hasSeededPose
                && inputs.hasEstimatedPose
                && inputs.estimatedPoseTimestamp != 0.0) {
                drivetrain.resetPose(inputs.estimatedPose);
                hasSeededPose = true;
                break;
            }
        }

        // Update each camera IO with its own inputs object
        for (int i = 0; i < cameras.size(); i++) {
            VisionIO io = cameras.get(i);
            VisionIOInputsAutoLogged inputs = cameraInputs.get(i);

            if (io instanceof VisionIOReal realIO) {
                realIO.setRobotYaw(yawDeg);
            } else if (io instanceof VisionIOSim simIO) {
                simIO.updateSimPose(driveState.Pose);
            }

            io.updateInputs(inputs);
        }

        // Clear tag maps for this cycle
        tagposes.clear();
        tagambiguities.clear();

        // Build tag maps from all cameras (only from cameras that currently have targets)
        for (var inputs : cameraInputs) {
            if (!inputs.hasTarget) continue;
            if (inputs.visibleTagIds == null || inputs.visibleTagPoses == null) continue;

            for (int i = 0; i < inputs.visibleTagIds.length; i++) {
                int id = inputs.visibleTagIds[i];
                Pose2d tagFieldPose = inputs.visibleTagPoses[i];
                Pose2d robotFieldPose = driveState.Pose;

                Pose2d tagRobotPose = tagFieldPose.relativeTo(robotFieldPose);

                tagposes.put(id, tagRobotPose);

                double ambiguity = inputs.hasEstimatedPose && inputs.estimatedPoseTimestamp != 0.0
                    ? 1.0 / Math.max(1, inputs.numTagsUsed)
                    : Double.MAX_VALUE;
                tagambiguities.put(id, ambiguity);
            }
        }

        // Collect good pose estimates from all cameras
        List<Pose2d> goodPoses = new ArrayList<>();
        double fusedTimestamp = 0.0;

        for (var inputs : cameraInputs) {
            if (!inputs.hasEstimatedPose) continue;
            if (inputs.estimatedPoseTimestamp == 0.0) continue; // fallback → ignore
            if (inputs.numTagsUsed <= 0) continue;
            if (!isEstimateValid(inputs.estimatedPose, yawDeg, inputs.estimatedPoseTimestamp)) continue;

            goodPoses.add(inputs.estimatedPose);
            fusedTimestamp = inputs.estimatedPoseTimestamp;
        }

        // Compute std devs based on closest tag distance and number of good poses
        if (!goodPoses.isEmpty()) {
            double closest = getClosestTagDistance(driveState.Pose);
            if (closest < 1.0) closest = 1.0;

            double stdDev = 0.04 * closest * closest / goodPoses.size() + 0.25;
            visionStdDevs = VecBuilder.fill(stdDev, stdDev, Double.MAX_VALUE);
        } else {
            visionStdDevs = VecBuilder.fill(
                Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
            );
        }

        // Fuse pose (real robot only)
        if (!goodPoses.isEmpty() && RobotBase.isReal()) {
            Pose2d fusedPose = fusePoses(goodPoses);
            drivetrain.addVisionMeasurement(
                fusedPose,
                Utils.fpgaToCurrentTime(fusedTimestamp),
                visionStdDevs
            );
        }

        // Log all camera inputs separately
        for (int i = 0; i < cameraInputs.size(); i++) {
            //Logger.processInputs("LoggedVision" + i, cameraInputs.get(i));
        }
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
            new edu.wpi.first.math.geometry.Rotation2d(theta / n)
        );
    }

    private boolean isEstimateValid(Pose2d pose, double headingDeg, double timestampSeconds) {
        if (pose == null) return false;

        double age = Timer.getFPGATimestamp() - timestampSeconds;
        if (age > 0.25) return false;

        if (pose.getX() < 0 || pose.getX() > Constants.FIELD_MAX_X) return false;
        if (pose.getY() < 0 || pose.getY() > Constants.FIELD_MAX_Y) return false;

        double headingError = Math.abs(MathUtil.inputModulus(
            pose.getRotation().getDegrees() - headingDeg,
            -180, 180
        ));
        return headingError <= 90.0;
    }

    private double getClosestTagDistance(Pose2d robotPose) {
        double min = Double.MAX_VALUE;

        for (var inputs : cameraInputs) {
            if (inputs.visibleTagPoses == null) continue;

            for (Pose2d tagPose : inputs.visibleTagPoses) {
                double d = robotPose.getTranslation().getDistance(tagPose.getTranslation());
                if (d < min) min = d;
            }
        }

        return min;
    }

    // *Getters
    public boolean hasTargets() {
        for (var in : cameraInputs) {
            if (in.hasTarget) return true;
        }
        return false;
    }

    public boolean hasTarget(int id) {
        return tagposes.containsKey(id);
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

    public Pose2d getEstimatedPose() {
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

    // *Helpers
    public boolean isAlignedToHub(Pose2d robotPose) {
        if (cachedHubPose == null) return false;

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

    public Transform2d getTransformToTag(int id) {
        return hasTarget(id)
            ? tagposes.get(id).minus(Pose2d.kZero)
            : Transform2d.kZero;
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return visionStdDevs;
    }

    public Pose2d getCachedHubPose() {
        return cachedHubPose;
    }

    // *Used in commands and elsewhere
    public double getRobotYawDeg() {
        return drivetrain.getState().Pose.getRotation().getDegrees();
    }

    public double getYawToTarget(Pose2d targetPose) {
        if (targetPose == null) return 0.0;

        Pose2d robotPose = drivetrain.getState().Pose;

        double angleToTarget = Math.toDegrees(Math.atan2(
            targetPose.getY() - robotPose.getY(),
            targetPose.getX() - robotPose.getX()
        ));

        Logger.recordOutput("AlignToHub/robotPoseX", robotPose.getX());
        Logger.recordOutput("AlignToHub/robotPoseY", robotPose.getY());
        Logger.recordOutput("AlignToHub/robotHeadingDeg", robotPose.getRotation().getDegrees());
        Logger.recordOutput("AlignToHub/targetPoseX", targetPose.getX());
        Logger.recordOutput("AlignToHub/targetPoseY", targetPose.getY());
        Logger.recordOutput("AlignToHub/angleToTargetDeg", angleToTarget);

        return MathUtil.inputModulus(
            angleToTarget - robotPose.getRotation().getDegrees(),
            -180, 180
        );
    }

    public double getDistanceToTarget(Pose2d targetPose) {
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

    // *Miscellaneous
    private void refreshAllianceCache() {
        if (cachedHubPose != null) return;
        if (DriverStation.getAlliance().isEmpty()) return;

        cachedHubPose = VisionConstants.getHubPose(
            DriverStation.getAlliance().get()
        );
    }
}
