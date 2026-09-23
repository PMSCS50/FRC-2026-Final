package frc.robot.subsystems.vision2;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {

    // *Create fields
    private final List<VisionIO> cameras;
    private final List<VisionIOInputsAutoLogged> cameraInputs = new ArrayList<>();
    private final CommandSwerveDrivetrain drivetrain;

    private final Debouncer alignDebouncer = new Debouncer(0.1, DebounceType.kBoth);

    private final HashMap<Integer, Pose2d> tagposes = new HashMap<>();

    public Pose2d cachedHubPose = null;
    public boolean hasSeededPose = false;
    private boolean autoStarted = false;

    private Pose2d[] poseArray = new Pose2d[0];

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
        robotPose = drivetrain.getPose();
        double yawDeg = robotPose.getRotation().getDegrees();

        tagposes.clear();

        double[] camDistBuf = new double[cameraInputs.size()];
        
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
                if (inputs.numTagsUsed < 1) continue;

                double age = Timer.getFPGATimestamp() - inputs.estimatedPoseTimestamp;
                if (age > 0.20) continue;

                double dist = camDistBuf[i];
                if (dist > 5.0) continue;

                double amb = inputs.ambiguity[1];
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

            // Update IO
            if (io instanceof VisionIOReal realIO) {
                realIO.setRobotYaw(yawDeg);
            } else if (io instanceof VisionIOSim simIO) {
                simIO.updateSimPose(robotPose);
            }

            io.updateInputs(inputs);

            // Tag processing
            double closestTagDist = Double.MAX_VALUE;

            if (inputs.hasTarget &&
                inputs.visibleTagIds != null &&
                inputs.visibleTagPoses != null) {

                Pose2d[] tagPoses = inputs.visibleTagPoses;
                int[] tagIds = inputs.visibleTagIds;

                for (int j = 0; j < tagIds.length; j++) {
                    int id = tagIds[j];
                    Pose2d tagRobotPose = tagPoses[j];

                    tagposes.put(id, tagRobotPose);

                    double d = tagRobotPose.getTranslation().getNorm();

                    if (d < closestTagDist) {
                        closestTagDist = d;
                    }
                }
            }

            if (!isEstimateValid(
                inputs.estimatedPose,
                yawDeg,
                inputs.estimatedPoseTimestamp)) {
                continue;
            }

            drivetrain.addVisionMeasurement(
                inputs.estimatedPose,
                Utils.fpgaToCurrentTime(inputs.estimatedPoseTimestamp),
                VecBuilder.fill(inputs.stdDevs[0], inputs.stdDevs[1], inputs.stdDevs[2])
            );

            Logger.processInputs("LoggedVision/" + io.getName(), inputs);
        }

        poseArray = tagposes.values().toArray(Pose2d[]::new);

        for (int i = 0; i < poseArray.length; i++) {
            poseArray[i] = robotPose.plus(new Transform2d(poseArray[i].getTranslation(), poseArray[i].getRotation()).inverse());
        }

        Logger.recordOutput("Vision/TagFieldPoses", poseArray);
        
    }


    // *Helpers
    private void refreshAllianceCache() {
        if (cachedHubPose != null) return;
        if (DriverStation.getAlliance().isEmpty()) return;

        cachedHubPose = VisionConstants.getHubPose(
            DriverStation.getAlliance().get()
        );
    }


    private boolean isEstimateValid(Pose2d estimatedPose, double headingDeg, double timestampSeconds) {
        if (estimatedPose == null) return false;

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
                double amb = in.ambiguity[0];
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

    // |Whether the robot is aligned to a certain pose with given tolerance in degrees
    public boolean isAlignedToPose(Pose2d target, double toleranceDeg) {
        Pose2d robotPose = drivetrain.getState().Pose;

        double angleToHub = Math.toDegrees(Math.atan2(
            target.getY() - robotPose.getY(),
            target.getX() - robotPose.getX()
        ));

        double yawErrorDeg = MathUtil.inputModulus(
            angleToHub - robotPose.getRotation().getDegrees(), -180, 180
        );

        return alignDebouncer.calculate(
            Math.abs(yawErrorDeg) <= toleranceDeg
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

        return MathUtil.angleModulus(
            angleToTarget - robotPose.getRotation().getDegrees()
        ) * 180 / Math.PI;
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
