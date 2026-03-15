package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Vision subsystem refactored for AdvantageKit.
 *
 * All hardware/sim interaction is delegated to a VisionIO implementation:
 *   - VisionIOReal  → real robot (PhotonCamera over NT)
 *   - VisionIOSim   → simulation (VisionSystemSim driven by drivetrain sim pose)
 *
 * This class only reads from the logged VisionIOInputsAutoLogged snapshot,
 * making it fully replay-safe.
 */
public class VisionSimSystem extends SubsystemBase {

    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final CommandSwerveDrivetrain drivetrain;

    // Cached std devs as a Matrix for drivetrain.addVisionMeasurement()
    private Matrix<N3, N1> visionStdDevs = VecBuilder.fill(
        Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
    );

    /**
     * Construct with explicit IO (preferred for unit testing / flexibility).
     *
     * Example in RobotContainer:
     *   var vision = new VisionSimSystem(
     *       RobotBase.isReal()
     *           ? new VisionIOReal("FrontCamera")
     *           : new VisionIOSim("FrontCamera"),
     *       drivetrain
     *   );
     */
    public VisionSimSystem(CommandSwerveDrivetrain drivetrain, VisionIO io) {
        this.io = io;
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // If running in simulation, push the true robot pose into VisionSystemSim
        // so it can synthesize realistic tag detections for this frame.
        if (!RobotBase.isReal() && io instanceof VisionIOSim simIO) {
            simIO.updateSimPose(drivetrain.getState().Pose);
        }

        // Pull all sensor data into the logged inputs snapshot.
        // AdvantageKit records every field of inputs to the log automatically.
        io.updateInputs(inputs);

        io.updateInputs(inputs);

        // All recordOutput calls first
        List<Pose2d> visibleTagPoses = new ArrayList<>();
        if (VisionConstants.aprilTagLayout != null) {
            for (int id : inputs.visibleTagIds) {
                VisionConstants.aprilTagLayout
                    .getTagPose(id)
                    .ifPresent(pose -> visibleTagPoses.add(pose.toPose2d()));
            }
        }
        Logger.recordOutput("Vision/VisibleTagCount", visibleTagPoses.size());
        Logger.recordOutput("Vision/VisibleTags", visibleTagPoses.toArray(new Pose2d[0]));

        // processInputs always last
        Logger.processInputs("Vision", inputs); 

        // Rebuild the std-dev Matrix from the logged double array
        visionStdDevs = VecBuilder.fill(
            inputs.visionStdDevs[0],
            inputs.visionStdDevs[1],
            inputs.visionStdDevs[2]
        );

        // Feed pose estimate into drivetrain if valid
        if (inputs.hasEstimatedPose) {
            drivetrain.addVisionMeasurement(
                inputs.estimatedPose,
                inputs.estimatedPoseTimestamp,
                visionStdDevs
            );
        }
    }


    // ========================
    // PUBLIC GETTERS
    // (all read from logged inputs — replay-safe)
    // ========================

    /** True if the camera sees the specified AprilTag ID this frame. */
    public boolean hasTarget(int desiredId) {
        return inputs.hasTarget && inputs.targetId == desiredId && inputs.hasTagTransform;
    }

    /** True if the camera sees any target this frame. */
    public boolean hasTargets() {
        return inputs.hasTarget;
    }

    /** Best visible tag ID, or -1 if none. */
    public int getTargetId() {
        return inputs.targetId;
    }

    /** Forward/back distance from robot to tag face (meters). */
    public double getX() {
        return inputs.hasTagTransform ? inputs.tagToRobotX : 0.0;
    }

    /** Left/right distance from robot to tag face (meters). */
    public double getY() {
        return inputs.hasTagTransform ? inputs.tagToRobotY : 0.0;
    }

    /** Robot yaw relative to tag (radians). */
    public double getYawRad() {
        if (!inputs.hasTagTransform) return 0.0;
        return MathUtil.angleModulus(inputs.tagToRobotRotZ - Math.PI);
    }

    /** Straight-line distance from robot to tag (meters). */
    public double getDistance() {
        return inputs.hasTagTransform
            ? Math.hypot(inputs.tagToRobotX, inputs.tagToRobotY)
            : 0.0;
    }

    /** Current vision measurement standard deviations. */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return visionStdDevs;
    }


    // ========================
    // SHOOTER RPM CALCULATION
    // (pure math — no IO dependency, unchanged from original)
    // ========================

    private final double shooterHeight = 0.508;
    private final double phi = Math.toRadians(70);

    public double rpmFromDistance(double distance) {
        double y = 1.8288 - shooterHeight;

        double shooterVelocity = Math.sqrt(
            (9.807 * distance * distance) /
            (2 * Math.cos(phi) * Math.cos(phi) * (distance * Math.tan(phi) + shooterHeight - y))
        );

        double dragFactor = (1 + 0.015 * distance) * 1.04;
        shooterVelocity *= dragFactor;

        double wheelRadius = 0.0508;
        double c = 1.0;
        return c * (shooterVelocity * 60.0) / (2.0 * Math.PI * wheelRadius) / 100;
    }
}