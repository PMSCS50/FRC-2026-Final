package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Transform3d;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.Constants.VisionConstants;

import java.util.Arrays;
import java.util.HashMap;
import java.util.stream.Stream;

// 2-camera Limelight subsystem.
// llCamera1: front-facing camera (0 deg yaw offset)
// llCamera2: rear-facing camera  (180 deg yaw offset)
// Fuses pose estimates from both cameras in the drivetrain's Kalman filter.

public class LLSubsystemNew extends VisionGeneral implements VisionIO {

    private final CommandSwerveDrivetrain drivetrain;
    private final String llCamera1; // front camera: peepee peeper
    private final String llCamera2; // rear camera: ass tumor

    private final HashMap<Integer, Transform2d> tagtransforms = new HashMap<>();

    private double omegaRps;

    //Pose of the robot, wrapped in latestEstimate
    private Pose2d estimatedRobotPose;
    private PoseEstimate latestEstimate;


    //Vision standard deviations
    private static final double BASE_XY_STD_DEV      = 0.5;
    private static final double THETA_STD_DEV        = 9999.0;
    private static final double MAX_AMBIGUITY        = 0.9;
    private static final double MAX_LATENCY_SECONDS  = 0.25;
    private static final double MAX_OMEGA_RPS        = 2.0;
    private static final double FIELD_MAX_X          = 16.5;
    private static final double FIELD_MAX_Y          = 8.5;

    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public LLSubsystemNew(CommandSwerveDrivetrain drivetrain, String llCamera1, String llCamera2) {
        this.drivetrain = drivetrain;
        this.llCamera1  = llCamera1;
        this.llCamera2  = llCamera2;

        setPipeline(9); 
        setIMUMode(4);

        //Clear tag filters
        LimelightHelpers.SetFiducialIDFiltersOverride(llCamera1, new int[]{});
        LimelightHelpers.SetFiducialIDFiltersOverride(llCamera2, new int[]{});
    }


    @Override
    public void periodic() {
        var driveState = drivetrain.getState();

        double headingDeg = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation(llCamera1, headingDeg, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(llCamera2, headingDeg, 0, 0, 0, 0, 0); 

        PoseEstimate llMeasurement1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llCamera1);
        PoseEstimate llMeasurement2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llCamera2);

        // Reset each loop to ensure we don't accidentally use stale data if both cameras are invalid
        estimatedRobotPose = null;
        latestEstimate     = null;

        boolean cam1Valid = isEstimateValid(llMeasurement1);
        boolean cam2Valid = isEstimateValid(llMeasurement2);

        // Send camera1 pose estimate
        if (cam1Valid) {
            Matrix<N3, N1> stdDevs = calculateStdDevs(llMeasurement1);
            if (stdDevs != null) {
                drivetrain.addVisionMeasurement(
                    llMeasurement1.pose,
                    Utils.fpgaToCurrentTime(llMeasurement1.timestampSeconds)//,
                    //stdDevs
                );
            }
        }

        // send camera2 po2585se estimate ,
        if (cam2Valid) {
            Matrix<N3, N1> stdDevs = calculateStdDevs(llMeasurement2);
            if (stdDevs != null) {
                drivetrain.addVisionMeasurement(
                    llMeasurement2.pose,
                    Utils.fpgaToCurrentTime(llMeasurement2.timestampSeconds)//,
                    //stdDevs
                );
            }
        }

        // Wrap estimatedRobotPose inside a PoseEstimate for more metadata.
        if (cam1Valid || cam2Valid) {
            estimatedRobotPose = drivetrain.getState().Pose;

            Logger.recordOutput("Vision/Front Cam PE Odometry PE difference magnitude", Math.hypot(
                llMeasurement1.pose.getX() - drivetrain.getState().Pose.getX(),
                llMeasurement1.pose.getY() - drivetrain.getState().Pose.getY()
            ));

            Logger.recordOutput("Vision/Back Cam PE Odometry PE difference magnitude", Math.hypot(
                llMeasurement2.pose.getX() - drivetrain.getState().Pose.getX(),
                llMeasurement2.pose.getY() - drivetrain.getState().Pose.getY()
            ));


            double avgTimestamp = (llMeasurement1.timestampSeconds + llMeasurement2.timestampSeconds) / 2.0;
            double avgLatency = (llMeasurement1.latency + llMeasurement2.latency) / 2.0;
            double avgTagSpan = (llMeasurement1.tagSpan + llMeasurement2.tagSpan) / 2.0;

            // Total unique tags seen across both cameras (no double-counting overlaps)
            RawFiducial[] totalTagsUsed = totalTagsUsed(llMeasurement1, llMeasurement2);
            int totalTags = totalTagsUsed.length;

            //Fused metadata from both pose estimates.
            latestEstimate = new PoseEstimate(
                estimatedRobotPose,
                avgTimestamp,
                avgLatency,
                totalTags,
                avgTagSpan,
                averageTagDistance(totalTagsUsed),
                averageTagArea(totalTagsUsed),
                totalTagsUsed,     
                true // always MegaTag2
            );
        }

        //Reset tagtransforms every periodic
        tagtransforms.clear();

        LimelightTarget_Fiducial[] allTags = allVisibleTags(
            LimelightHelpers.getLatestResults(llCamera1).targets_Fiducials,
            LimelightHelpers.getLatestResults(llCamera2).targets_Fiducials
        );

        //Tag HashMap thing Kevin did
        for (LimelightTarget_Fiducial fiducial : allTags) {
            Pose2d pose = fiducial.getRobotPose_TargetSpace().toPose2d();
            Transform2d tagToRobot = new Transform2d(
                pose.getX(),
                pose.getY(),
                pose.getRotation()
            );
            tagtransforms.put(
                (int) fiducial.fiducialID,
                tagToRobot
            );
        }

        //AdvantageKit Logging 
        Logger.recordOutput("Vision/Heading Sent to LL",    headingDeg);
        Logger.recordOutput("Vision/Raw Pigeon Yaw",        drivetrain.getPigeon2().getYaw().getValueAsDouble());
        Logger.recordOutput("Vision/Omega RPS",             omegaRps);
        Logger.recordOutput("Vision/Cam1 Tag Count",        llMeasurement1 != null ? llMeasurement1.tagCount : 0);
        Logger.recordOutput("Vision/Cam2 Tag Count",        llMeasurement2 != null ? llMeasurement2.tagCount : 0);
        Logger.recordOutput("Vision/Cam1 Valid",            cam1Valid);
        Logger.recordOutput("Vision/Cam2 Valid",            cam2Valid);

        updateInputs(inputs);
        Logger.processInputs("LoggedVision", inputs);
    }

    //Validation

    private boolean isEstimateValid(PoseEstimate estimate) {
        if (estimate == null || estimate.tagCount == 0) return false;

        double ageSeconds = Utils.fpgaToCurrentTime(0)
                          - Utils.fpgaToCurrentTime(estimate.timestampSeconds);
        if (ageSeconds > MAX_LATENCY_SECONDS) return false;

        if (Math.abs(omegaRps) > MAX_OMEGA_RPS) return false;

        Pose2d pose = estimate.pose;
        if (pose.getX() < 0 || pose.getX() > FIELD_MAX_X) return false;
        if (pose.getY() < 0 || pose.getY() > FIELD_MAX_Y) return false;

        return true;
    }

    // Vision STDs

    //This one was made by Big C bc aint no way I'm creating a system for stdDevs myself
    private Matrix<N3, N1> calculateStdDevs(PoseEstimate estimate) {
        if (estimate == null || estimate.tagCount == 0) return VecBuilder.fill(9999.0, 9999.0, 9999.0);

        double xyStdDev = BASE_XY_STD_DEV;

        xyStdDev /= (0.35*estimate.tagCount + 0.65*estimate.avgTagDist);
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


    //Used to count tags for POSE ESTIMATION
    private RawFiducial[] totalTagsUsed(PoseEstimate est1, PoseEstimate est2) {
        if (est1 == null && est2 == null) return new RawFiducial[0];
        if (est1 != null && est2 == null) return est1.rawFiducials;
        if (est1 == null && est2 != null) return est2.rawFiducials;

        RawFiducial[] combined = Stream.of(est1.rawFiducials, est2.rawFiducials)
                                       .flatMap(Arrays::stream)
                                       .distinct()
                                       .toArray(RawFiducial[]::new);
        return combined;
    }

    //Used to count tags for TAG DATA
    private LimelightTarget_Fiducial[] allVisibleTags(LimelightTarget_Fiducial[] t1, LimelightTarget_Fiducial[] t2) {

        if (t1 == null && t2 == null) return new LimelightTarget_Fiducial[0];
        if (t1 != null && t2 == null) return t1;
        if (t1 == null && t2 != null) return t2;

        //Removes all duplicates from the array.
        LimelightTarget_Fiducial[] allTags = Stream.of(t1, t2)
                                                   .flatMap(Arrays::stream)
                                                   .distinct()
                                                   .toArray(LimelightTarget_Fiducial[]::new);

        return allTags;
    }

    private double averageTagDistance(RawFiducial[] fiducials) {
        if (fiducials == null || fiducials.length == 0) return -1.0;
        double sum = 0;
        for (RawFiducial tag : fiducials) sum += tag.distToRobot;
        return sum / fiducials.length;
    }

    private double averageTagArea(RawFiducial[] fiducials) {
        if (fiducials == null || fiducials.length == 0) return -1.0;
        double sum = 0;
        for (RawFiducial tag : fiducials) sum += tag.ta;
        return sum / fiducials.length;
    }

    private double avgAmbiguity(PoseEstimate estimate) {
        if (estimate.rawFiducials == null || estimate.rawFiducials.length == 0) return 1.0;
        double sum = 0;
        for (RawFiducial f : estimate.rawFiducials) sum += f.ambiguity;
        return sum / estimate.rawFiducials.length;
    }

    //getters

    public Pose2d  getPose()        { return estimatedRobotPose; }
    public double  getX(int i)           { return estimatedRobotPose != null ? estimatedRobotPose.getX() : 0.0; }
    public double  getY(int i)           { return estimatedRobotPose != null ? estimatedRobotPose.getY() : 0.0; }
    public double  getYaw()         { return estimatedRobotPose != null ? estimatedRobotPose.getRotation().getDegrees() : 0.0; }
    public double  getYawRad(int i)      { return estimatedRobotPose != null ? estimatedRobotPose.getRotation().getRadians() : 0.0; }
    public int     getTagCount()    { return latestEstimate != null ? latestEstimate.tagCount : 0; }
    public double  getAvgTagDist()  { return latestEstimate != null ? latestEstimate.avgTagDist : 0.0; }
    public double  getAvgTagArea()  { return latestEstimate != null ? latestEstimate.avgTagArea : 0.0; }
    public boolean hasTargets()     { return LimelightHelpers.getTV(llCamera1) || LimelightHelpers.getTV(llCamera2); }
    //public boolean hasTarget(int i)      { return hasTargets(); } // alias for compatibility with VisionGeneral

    public double getTagX(int id) {
        return hasTarget(id) ? tagtransforms.get(id).getX() : 0.0;
    }

    public double getTagY(int id) {
        return hasTarget(id) ? tagtransforms.get(id).getY() : 0.0;    
    }

    public double getTagYaw(int id) {
        return hasTarget(id) ? tagtransforms.get(id).getRotation().getRadians() : 0.0;    
    }

    public Transform2d getTransformToTag(int id) {
        return hasTarget(id) ? tagtransforms.get(id) : Transform2d.kZero;    
    }
    
    
    public boolean hasTarget(int desiredId) {
        return hasFiducial(llCamera1, desiredId) || hasFiducial(llCamera2, desiredId);
    }

    private boolean hasFiducial(String cameraName, int id) {
        return LimelightHelpers.getFiducialID(cameraName) == id;
    }

    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(llCamera1, pipeline);
        LimelightHelpers.setPipelineIndex(llCamera2, pipeline);
    }

    public void setIMUMode(int mode) {
        LimelightHelpers.SetIMUMode(llCamera1, mode);
        LimelightHelpers.SetIMUMode(llCamera2, mode);
    }

    // distance utilities

    public double getDistanceToTarget(Pose2d targetPose) {
        if (!hasTargets()) return -1.0;
        return estimatedRobotPose.getTranslation().getDistance(targetPose.getTranslation());
    }

    public double getYawToTarget(Pose2d targetPose) {
        if (!hasTargets()) return -1.0;
        return estimatedRobotPose.minus(targetPose).getRotation().getRadians();
    }

    /**
     * Returns direct tag distance from rawFiducials of the best camera reading.
     */
    public double getDistanceToTag(int tagId) {
        if (latestEstimate == null || latestEstimate.rawFiducials == null) return -1.0;
        for (RawFiducial fiducial : latestEstimate.rawFiducials) {
            if (fiducial.id == tagId) return fiducial.distToRobot;
        }
        return -1.0;
    }

    // For the closest (primary) tag, mainly to make it usable for VisionGeneral
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
   
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.hasTarget       = hasTargets();
        inputs.targetId        = hasTargets() ? (int) LimelightHelpers.getFiducialID(llCamera1) : -1; // best-effort primary target ID
        inputs.hasTagTransform = inputs.hasTarget;

        inputs.tagToRobotX    = inputs.hasTagTransform ? getTagX(inputs.targetId) : 0.0;
        inputs.tagToRobotY    = inputs.hasTagTransform ? getTagY(inputs.targetId) : 0.0;
        inputs.tagToRobotZ    = 0.0; // Limelight's fiducial pose does not provide Z translation, so we set it to 0 for the primary tag as well.
        inputs.tagToRobotRotZ = inputs.hasTagTransform ? getTagYaw(inputs.targetId) : 0.0;

        inputs.visibleTagIds     = tagtransforms.keySet().stream().mapToInt(Integer::intValue).toArray();
        inputs.visibleTagPoses   = Arrays.stream(inputs.visibleTagIds)
                                  .filter(tagtransforms::containsKey)
                                  .mapToObj(id -> new Pose2d(getTagX(id), getTagY(id), new Rotation2d(getTagYaw(id))))
                                  .toArray(Pose2d[]::new);
                                  
        Logger.recordOutput("Vision/VisibleTagPoses", inputs.visibleTagPoses);

        inputs.allTagToRobotX    = Arrays.stream(inputs.visibleTagIds).mapToDouble(this::getTagX).toArray();
        inputs.allTagToRobotY    = Arrays.stream(inputs.visibleTagIds).mapToDouble(this::getTagY).toArray();
        inputs.allTagToRobotZ    = new double[inputs.visibleTagIds.length]; // Limelight's fiducial pose does not provide Z translation, so we set it to 0 for all tags.    
        inputs.allTagToRobotRotZ = Arrays.stream(inputs.visibleTagIds).mapToDouble(this::getTagYaw).toArray();

        inputs.hasEstimatedPose = estimatedRobotPose != null;

        inputs.estimatedPose = estimatedRobotPose != null ? estimatedRobotPose : new Pose2d();

        inputs.estimatedPoseTimestamp = latestEstimate != null ? latestEstimate.timestampSeconds : 0.0;

        inputs.numTagsUsed = latestEstimate != null ? latestEstimate.tagCount : 0;

        inputs.avgTagDistMeters = latestEstimate != null ? latestEstimate.avgTagDist : 0.0;

        Matrix<N3, N1> stdDevMatrix = calculateStdDevs(latestEstimate);
        inputs.visionStdDevs = new double[]{stdDevMatrix.get(0, 0), stdDevMatrix.get(1, 0), stdDevMatrix.get(2, 0)};

         }

}