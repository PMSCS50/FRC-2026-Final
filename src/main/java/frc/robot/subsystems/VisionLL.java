package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

//Limelight VisionSubsystem with MegaTag2. 
//Organized the methods of LimelightHelpers into a more cohesive subsystem.
//and added some utility methods for accessing tag transforms and pose estimates.

public class VisionLL extends SubsystemBase {

    private static final double CAM_FORWARD_M  =  0.072;   // meters
    private static final double CAM_SIDE_M     =  -.072;  // meters (positive = left)
    private static final double CAM_UP_M       =  0.495;   // meters
    private static final double CAM_ROLL_DEG   =  0.0;
    private static final double CAM_PITCH_DEG  =  -Math.toRadians(10);
    private static final double CAM_YAW_DEG    =  0.0;

    private final String limelightName;
    private final CommandSwerveDrivetrain drivetrain;

    private boolean hasTarget = false;

    private int targetId = -1;


    private Transform3d tagToRobot = null;

    private Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0, 0, 0);


    private final HashMap<Integer, Transform3d> tagTransforms = new HashMap<>();


    public LimelightVisionSubsystem(String limelightName, CommandSwerveDrivetrain drivetrain) {
        this.limelightName = limelightName;
        this.drivetrain = drivetrain;

        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName,
            CAM_FORWARD_M, CAM_SIDE_M, CAM_UP_M,
            CAM_ROLL_DEG,  CAM_PITCH_DEG, CAM_YAW_DEG
        );
    }

    @Override
    public void periodic() {


        // MegaTag2 PoseEstimation requires an up-to-date robot orientation BEFORE the pose
        // estimate is read, otherwise the firmware uses a stale heading and
        // translation accuracy degrades
        double currentYawDeg = drivetrain.getState().Pose.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(
            limelightName,
            currentYawDeg,
            0, 0, 0, 0, 0  // yaw rate / pitch / roll not required
        );

        hasTarget = LimelightHelpers.getTV(limelightName);

        if (!hasTarget) {
            targetId   = -1;
            tagToRobot = null;
            tagTransforms.clear();
            return;
        }

        targetId = (int) LimelightHelpers.getFiducialID(limelightName);


        Pose3d tagToRobotPose = LimelightHelpers.getBotPose3d_TargetSpace(limelightName);
        tagToRobot = new Transform3d(
            tagToRobotPose.getTranslation(),
            tagToRobotPose.getRotation()
        );

        tagTransforms.clear();
        LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
        for (LimelightTarget_Fiducial target : results.targets_Fiducials) {
            int id = (int) target.fiducialID;
            Pose3d robotPoseInTagSpace = target.getRobotPose_TargetSpace();
            tagTransforms.put(id, new Transform3d(
                robotPoseInTagSpace.getTranslation(),
                robotPoseInTagSpace.getRotation()
            ));
        }

        // --- 6. MegaTag2 field pose + std-devs → drivetrain ------------------
        Optional<PoseEstimate> estimatedRobotPose = estimateFieldPose();

        estimatedRobotPose.ifPresent(erp -> {
            updateEstimationStdDevs(erp);
            drivetrain.addVisionMeasurement(
                erp.pose,
                erp.timestampSeconds,
                visionStdDevs
            );
        });
    }


    public boolean hasTargets() {
        return hasTarget;
    }

    public boolean hasTarget(int desiredId) {
        return hasTarget && targetId == desiredId && tagToRobot != null;
    }

    public int getTargetId() {
        return targetId;
    }

    public Transform3d getBestTagtoRobot() {
        return tagToRobot;
    }

    public double getX() {
        return tagToRobot != null ? tagToRobot.getX() : 0.0;
    }

    public double getY() {
        return tagToRobot != null ? tagToRobot.getY() : 0.0;
    }

    public double getYawRad() {
        return tagToRobot != null ? tagToRobot.getRotation().getZ() : 0.0;
    }

    public double getX(int id) {
        Transform3d transform = tagTransforms.get(id);
        return transform != null ? transform.getX() : 0.0;
    }

    public double getY(int id) {
        Transform3d transform = tagTransforms.get(id);
        return transform != null ? transform.getX() : 0.0;
    }

    public double getYawRad(int id) {
        Transform3d transform = tagTransforms.get(id);
        return transform != null ? transform.getX() : 0.0;
    }

    public Map<Integer, Transform3d> getTagTransforms() {
        return Collections.unmodifiableMap(tagTransforms);
    }

    public Optional<Transform3d> getTagTransform(int tagId) {
        return Optional.ofNullable(tagTransforms.get(tagId));
    }

    public boolean hasTagTransform(int tagId) {
        return tagTransforms.containsKey(tagId);
    }
    public double getDistance(int id) {
        Transform3d tag = tagTransforms.get(id);
        return tag != null ? Math.hypot(tag.getX(), tag.getY()) : 0.0;
    }

    public Optional<PoseEstimate> estimateFieldPose() {
        double headingDeg = drivetrain.getPose().getRotation().getDegrees()
        LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
        PoseEstimate pe = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (!LimelightHelpers.validPoseEstimate(pe)) {
            return Optional.empty();
        }

        return Optional.of(pe);
    }

    private void updateEstimationStdDevs(PoseEstimate pe) {
        int numTags = pe.tagCount;
        double avgDist = pe.avgTagDist;

        if (numTags >= 2) {
            visionStdDevs = VecBuilder.fill(
                0.5 * avgDist,
                0.5 * avgDist,
                Math.toRadians(5)
            );
        } else {
            visionStdDevs = VecBuilder.fill(
                1.0 * avgDist,
                1.0 * avgDist,
                Math.toRadians(10)            
            );
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return visionStdDevs;
    }

    //***************
    //SHOOTER HELPERS
    //***************
    
    double shooterHeight = 0.508;
    double phi = Math.toRadians(70);

    public double rpmFromDistanceRegression(double distance) {
        double rps = -0.3039109023 * distance * distance
                + 6.81380687 * distance
                + 40.82402705 - .5;
        SmartDashboard.putNumber("Shooter rps regression", rps);
        double rpm = rps * 60;
        SmartDashboard.putNumber("Shooter rpm regression", rpm);
        return rpm;
    }

    //Added my model back. My dumbass realized c is actually slip factor. We measure that empirically
    public double rpmFromDistance(double distance) {
        double height = 1.8288 - shooterHeight;

        double v = Math.sqrt(
            (9.807 * distance * distance) / 
            (2 * Math.cos(phi) * Math.cos(phi) * (distance * Math.tan(phi) - height))
        );

        double dragFactor = (1 + 0.0000001 * distance * distance) * 1.031;
        v *= dragFactor;

        double wheelRadius = 0.0508;
        double slip = 0.95;
        double wheelRPM = (v * 60.0) / (slip * Math.PI * wheelRadius);
        return wheelRPM;
    }



}