package frc.robot.subsystems.vision2;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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

        public Matrix<N3, N1> stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    // *Updates the set of loggable inputs. Called every loop in Vision.periodic()
    public void updateInputs(VisionIOInputs inputs);
     
}
