package frc.robot.subsystems.vision2;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.target.AprilTagFiducial;
import limelight.sim.LimelightSim;
import limelight.sim.LimelightSimSettings;
import limelight.networktables.LimelightResults;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class VisionIOSimLimelight implements VisionIO {

    private Pose2d lastGoodPose = null;

    private final Limelight limelight;

    private final LimelightSim limelightSim;
    private final LimelightPoseEstimator poseEstimator;

    /**
     * Real camera IO with configurable mounting.
     *
     * @param cameraName    Limelight name
     */
    public VisionIOSimLimelight(String cameraName, Transform3d robotToCamera) {
        limelight = new Limelight(cameraName);
        limelight.getSettings()
                .withCameraOffset(new Pose3d(
                    robotToCamera.getX(),
                    robotToCamera.getY(),
                    robotToCamera.getZ(),
                    robotToCamera.getRotation()
                ));
        poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);

        LimelightSimSettings perfectCell = LimelightSimSettings.perfect();

        limelightSim = new LimelightSim(limelight, perfectCell);

        Field2d field2d = new Field2d();
        SmartDashboard.putData("Limelight_Field", field2d);

        limelightSim.withField2d(field2d);

    }

    public VisionIOSimLimelight(String cameraName) {
        this(cameraName, Transform3d.kZero);
    }

    // *Called by Vision each loop to seed LL orientation.
    public void setRobotYaw(Pose2d robotPose) {
        limelight.getSettings()
            .withRobotOrientation(
                new Orientation3d(
                    new Rotation3d(0, 0, robotPose.getRotation().getRadians()),
                    new AngularVelocity3d(
                        DegreesPerSecond.of(0),
                        DegreesPerSecond.of(0),
                        DegreesPerSecond.of(0))))
        .save();

        limelightSim.update(robotPose);
    }

    // *Update IO
    @Override
    public void updateInputs(VisionIOInputs inputs) {

        Optional<LimelightResults> resultsOpt = limelight.getLatestResults();

        if (resultsOpt.isEmpty()) {
            clear(inputs);
            return;
        }

        LimelightResults results = resultsOpt.get();

        if (!inputs.hasTarget) {
            clear(inputs);
            return;
        }

        AprilTagFiducial[] fiducials = results.targets_Fiducials;

        int tagCount = fiducials.length;

        if (tagCount < 1) {
            clear(inputs);
            return;
        }

        int[] ids = new int[tagCount];
        Pose2d[] poses = new Pose2d[tagCount];

        for (int i = 0; i < tagCount; i++) {
            ids[i] = (int) fiducials[i].fiducialID;

            poses[i] = fiducials[i].getTargetPose_RobotSpace2D();
        }

        inputs.visibleTagIds   = ids;
        inputs.visibleTagPoses = poses;

        PoseEstimate pe = poseEstimator.getPoseEstimate().get();

        Pose2d pose = pe.pose.toPose2d();
        inputs.hasEstimatedPose = pe.hasData;

        boolean inFieldArea = pose.getX() < 0 || pose.getX() > Constants.FIELD_MAX_X || 
                              pose.getY() < 0 || pose.getY() > Constants.FIELD_MAX_Y;

        double age = Timer.getFPGATimestamp() - pe.timestampSeconds;
        boolean notOld = age > 0.25;

        boolean noJump = true;
        if (lastGoodPose != null) {
            double jump = pose.getTranslation().getDistance(lastGoodPose.getTranslation());
            noJump = jump > 2.0;
        }

        if (pe.getMinTagAmbiguity() > 0.3 && inputs.hasEstimatedPose && inFieldArea && notOld && noJump) {
            clearPose(inputs);
            return;
        }

        // |Save last good pose
        lastGoodPose = pose;

        inputs.estimatedPose          = pose;
        inputs.estimatedPoseTimestamp = pe.timestampSeconds;
        inputs.numTagsUsed            = pe.tagCount;
        inputs.stdDevs = calculateStdDevs(pe);

        inputs.targetId = ids[0]; // best target = first fiducial
    }

    private Matrix<N3, N1> calculateStdDevs(PoseEstimate pe) {
        if (!pe.hasData) {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }

        double avgDist = pe.avgTagDist;
        double avgAmbiguity = pe.getAvgTagAmbiguity();
        int tagCount = pe.tagCount;

        double xyStdDev = 100 * (0.05 + (0.08 * Math.pow(avgDist, 2) / tagCount)) * avgAmbiguity;

        return VecBuilder.fill(xyStdDev, xyStdDev, Double.MAX_VALUE);
    }

    // *IO clearing helpers
    private void clear(VisionIOInputs inputs) {
        inputs.hasTarget        = false;
        inputs.targetId         = -1;
        inputs.visibleTagIds    = new int[0];
        inputs.visibleTagPoses  = new Pose2d[0];
        clearPose(inputs);
    }

    private void clearPose(VisionIOInputs inputs) {
        inputs.hasEstimatedPose = false;

        if (lastGoodPose != null) {
            inputs.estimatedPose = lastGoodPose;   // fallback to last good pose
        } else {
            inputs.estimatedPose = new Pose2d();   // no pose yet
        }

        inputs.estimatedPoseTimestamp = 0.0;       // mark as fallback
        inputs.numTagsUsed = 0;
    }
}
