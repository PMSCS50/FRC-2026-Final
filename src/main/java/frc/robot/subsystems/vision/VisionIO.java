package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs {

        // *Primary (best) target for this camera
        public boolean hasTarget = false;
        public int targetId = -1;
        public boolean hasTagTransform = false;

        // *All visible targets for this camera
        public int[] visibleTagIds = new int[0];
        public Pose2d[] visibleTagPoses = new Pose2d[0];

        // *Pose estimation from this camera
        public boolean hasEstimatedPose = false;
        public Pose2d estimatedPose = new Pose2d();
        public double estimatedPoseTimestamp = 0.0;
        public int numTagsUsed = 0;
    }

    /** Updates the set of loggable inputs. Called every loop in Vision.periodic(). */
    public default void updateInputs(VisionIOInputs inputs) {}
}
