package frc.robot.util.pathfinding.zoro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

//Autologs Zoro inputs for compatibility with AdvantageKit log replay
public interface ZoroIO {

    @AutoLog
    public static class ZoroIOInputs {
        public boolean isNewPathAvailable = false;
        public Pose2d[] currentPathPoints = new Pose2d[0];
        public Rotation2d[] currentRotationTargets = new Rotation2d[0];
        public boolean[] hasRotationTarget = new boolean[0];
        public double[] waypointRelativePoses = new double[0];
        public double[] allConstraints = new double[0];
        public String[] eventTriggerNames = new String[0];
        public double[] eventPositions = new double[0];
        public double[] eventEndPositions = new double[0];
    }
    
}
