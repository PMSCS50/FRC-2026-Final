package frc.robot.subsystems.vision;

import java.util.HashMap;

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
import edu.wpi.first.wpilibj.Timer; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase{

    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final CommandSwerveDrivetrain drivetrain;

    private final Debouncer alignDebouncer = new Debouncer(0.1, DebounceType.kBoth);

    private final HashMap<Integer, Pose2d> tagposes = new HashMap<>();
    private final HashMap<Integer, Double> tagambiguities = new HashMap<>();

    public Pose2d cachedHubPose = null;
    public boolean hasSeededPose = false;

    private Matrix<N3, N1> visionStdDevs = VecBuilder.fill(
        Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
    );

    public Vision(CommandSwerveDrivetrain drivetrain, VisionIO io) {
        this.drivetrain = drivetrain;
        this.io         = io;
    }

    public void periodic() {

        refreshAllianceCache();

        if (!hasSeededPose && inputs.hasEstimatedPose) {
            drivetrain.resetPose(inputs.estimatedPose);
            hasSeededPose = true;
        }

        var driveState = drivetrain.getState();
        double yawDeg  = driveState.Pose.getRotation().getDegrees();

        // Seed LL or PhotonVision
        if (io instanceof VisionIOReal realIO) {
            realIO.setRobotYaw(yawDeg);
        } else if (io instanceof VisionIOSim simIO) {
            simIO.updateSimPose(driveState.Pose);
        }

        // Pull IO data
        io.updateInputs(inputs);

        tagposes.clear();
        tagambiguities.clear();

        // Build tag poses in robot frame
        if (inputs.visibleTagIds != null && inputs.visibleTagPoses != null) {
            for (int i = 0; i < inputs.visibleTagIds.length; i++) {
                int id = inputs.visibleTagIds[i];
                Pose2d tagFieldPose = inputs.visibleTagPoses[i];
                Pose2d robotFieldPose = driveState.Pose;

                Pose2d tagRobotPose = tagFieldPose.relativeTo(robotFieldPose);

                tagposes.put(id, tagRobotPose);
                tagambiguities.put(id, 0.0);
            }
        }

        // Compute std devs
        if (inputs.hasEstimatedPose && inputs.numTagsUsed > 0) {
            double closest = getClosestTagDistance(driveState.Pose);
            if (closest < 1.0) closest = 1.0;

            double stdDev = 0.04 * closest * closest / inputs.numTagsUsed + 0.25;
            visionStdDevs = VecBuilder.fill(stdDev, stdDev, Double.MAX_VALUE);
        } else {
            visionStdDevs = VecBuilder.fill(
                Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
            );
        }

        // Fuse pose
        if (inputs.hasEstimatedPose && isEstimateValid(inputs.estimatedPose, yawDeg, inputs.estimatedPoseTimestamp)) {
            drivetrain.addVisionMeasurement(
                inputs.estimatedPose,
                Utils.fpgaToCurrentTime(inputs.estimatedPoseTimestamp),
                visionStdDevs
            );
        }

        Logger.recordOutput("Vision/PoseEstimate", driveState.Pose);
        Logger.recordOutput("Vision/IsAlignedToHub", isAlignedToHub(driveState.Pose));

        Logger.processInputs("LoggedVision", inputs);
    }

    // -------------------------------------------------------------------------
    // Validation
    // -------------------------------------------------------------------------

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
        return headingError <= 90;
    }

    private double getClosestTagDistance(Pose2d robotPose) {
        double min = Double.MAX_VALUE;
        if (inputs.visibleTagPoses == null) return min;

        for (Pose2d tagPose : inputs.visibleTagPoses) {
            double d = robotPose.getTranslation().getDistance(tagPose.getTranslation());
            if (d < min) min = d;
        }
        return min;
    }

    // *Getters
    public boolean hasTargets() {
        return inputs.hasTarget;
    }

    public boolean hasTarget(int id) {
        return tagposes.containsKey(id);
    }

    public int getBestTargetId() {
        return inputs.targetId;
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
        return inputs.hasEstimatedPose ? inputs.estimatedPose : Pose2d.kZero;
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

        // If hub tag is visible, use tag distance
        if (hasTarget(hubTagId)) {
            return getDistance(hubTagId);
        }

        // Otherwise use estimated robot pose → hub pose
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
