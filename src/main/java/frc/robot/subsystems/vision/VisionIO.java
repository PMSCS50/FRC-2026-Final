package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs {
        // Whether any target is visible
        public boolean hasTarget = false;

        // Best target info
        public int targetId = -1;

        // Tag-relative transform (tagToRobot), decomposed for logging
        public double tagToRobotX = 0.0;
        public double tagToRobotY = 0.0;
        public double tagToRobotZ = 0.0;
        public double tagToRobotRotZ = 0.0; // yaw component

        // Whether we have a valid tagToRobot transform
        public boolean hasTagTransform = false;

        // Pose estimation output
        public boolean hasEstimatedPose = false;
        public Pose2d estimatedPose = new Pose2d();
        public double distanceToHub = 0;
        public double estimatedPoseTimestamp = 0.0;

        // Standard deviations for the estimated pose [x, y, theta]
        public double[] visionStdDevs = new double[] {
            Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
        };

        // Number of tags used in pose estimate
        public int numTagsUsed = 0;

        // Average distance to visible tags
        public double avgTagDistMeters = 0.0;

        // Expose full list (maybe)
        public int[] visibleTagIds = new int[0];
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {}
}
