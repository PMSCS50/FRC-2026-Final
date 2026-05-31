package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs {
        
        // !Primary (best) target
        //** Whether any AprilTag is visible this frame. */
        public boolean hasTarget = false;

        //** Fiducial ID of the best/primary target, or -1 if none. */
        public int targetId = -1;

        //** Whether tagToRobot for the primary target is valid. */
        public boolean hasTagTransform = false;

        /**
         * // !Primary tagToRobot transform, decomposed for AK logging.
         * // ?Represents the robot's pose in the primary tag's coordinate frame.
         */
        public double tagToRobotX    = 0.0;
        public double tagToRobotY    = 0.0;
        public double tagToRobotZ    = 0.0;
        public double tagToRobotRotZ = 0.0; // yaw (radians)

        // !All visible tags — parallel arrays (index-matched)
        // ?tagIds[i] corresponds to tagX[i], tagY[i], tagZ[i], tagRotZ[i]
        //** IDs of every currently visible AprilTag. */
        public int[] visibleTagIds = new int[0];
        public Pose2d[] visibleTagPoses = new Pose2d[0];

        /**
         * // *Per-tag tagToRobot transforms, decomposed into flat parallel arrays
         * // *so AK can log them. Index i corresponds to visibleTagIds[i].
         */
        public double[] allTagToRobotX    = new double[0];
        public double[] allTagToRobotY    = new double[0];
        public double[] allTagToRobotZ    = new double[0];
        public double[] allTagToRobotRotZ = new double[0];

        // !Pose estimation (MegaTag2)
        //** Whether a valid field-to-robot pose estimate is available. */
        public boolean hasEstimatedPose = false;

        //** Estimated robot pose in field (WPI Blue) coordinates. */
        public Pose2d estimatedPose = new Pose2d();

        //** FPGA timestamp of the pose estimate (seconds). */
        public double estimatedPoseTimestamp = 0.0;

        //** Number of tags used in the pose estimate. */
        public int numTagsUsed = 0;

        //** Average distance to visible tags used in pose estimate (meters). */
        public double avgTagDistMeters = 0.0;

        /**
         * // *Vision std-devs [x, y, theta] for addVisionMeasurement().
         * // *Set to Double.MAX_VALUE when no valid estimate is available.
         */
        public double[] visionStdDevs = new double[] {
            Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
        };


        // !Hub / aiming helpers
        /**
         * // *Straight-line distance from the robot to the hub (meters).
         * // *Computed from tag-space geometry — 0.0 if no tag is visible.
         */
        public double distanceToHub = 0.0;
    }

    /** Updates the set of loggable inputs. Called every loop in periodic(). */
    public default void updateInputs(VisionIOInputs inputs) {}
}