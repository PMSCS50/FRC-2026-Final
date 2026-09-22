package frc.robot.subsystems.vision2;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {

    // *IO interface
    @AutoLog
    public static class VisionIOInputs {

        // |Primary (best) target for this camera
        public boolean hasTarget = false;
        public int targetId = -1;
        public boolean hasTagTransform = false;

        // |All visible targets for this camera
        public int[] visibleTagIds = new int[0];
        public Pose2d[] visibleTagPoses = new Pose2d[0];

        // |Pose estimation from this camera
        public boolean hasEstimatedPose = false;
        public Pose2d estimatedPose = new Pose2d();
        public double estimatedPoseTimestamp = 0.0;
        public int numTagsUsed = 0;
        
        // [minAmbiguity, avgAmbiguity, maxAmbiguity]
        public double[] ambiguity = new double[3];

        // [x_stddev, y_stddev, yaw_stddev]
        public double[] stdDevs = {0.9, 0.9, Double.MAX_VALUE};
    }

    // *Updates the set of loggable inputs. Called every loop in Vision.periodic()
    public void updateInputs(VisionIOInputs inputs);

    // *Returns the Camera Name
    public String getName();
     
}
