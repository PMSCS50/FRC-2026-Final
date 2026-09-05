package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs {
        
        // *Primary (best) target
        public boolean hasTarget = false;
        public int targetId = -1;
        public boolean hasTagTransform = false;

        // public double tagToRobotX    = 0.0;
        // public double tagToRobotY    = 0.0;
        // public double tagToRobotZ    = 0.0;
        // public double tagToRobotRotZ = 0.0; // yaw (radians)

        // *All visible targets
        public int[] visibleTagIds = new int[0];
        public Pose2d[] visibleTagPoses = new Pose2d[0];

        // public double[] allTagToRobotX    = new double[0];
        // public double[] allTagToRobotY    = new double[0];
        // public double[] allTagToRobotZ    = new double[0];
        // public double[] allTagToRobotRotZ = new double[0];
        // public double avgTagDistMeters = 0.0;

        // *Pose estimation
        public boolean hasEstimatedPose = false;
        public Pose2d estimatedPose = new Pose2d();
        public double estimatedPoseTimestamp = 0.0;
        public int numTagsUsed = 0;

        //public double distanceToHub = 0.0;
        // public double[] visionStdDevs = new double[] {
        //     Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
        // };
    }

    /** Updates the set of loggable inputs. Called every loop in periodic(). */
    public default void updateInputs(VisionIOInputs inputs) {}
}