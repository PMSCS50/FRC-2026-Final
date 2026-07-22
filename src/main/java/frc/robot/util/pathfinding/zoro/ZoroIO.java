package frc.robot.util.pathfinding.zoro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

//Autologs Zoro inputs for compatibility with AdvantageKit log replay
public interface ZoroIO {

    @AutoLog
    public static class ZoroIOInputs {
        public boolean isNewPathAvailable;
        public Pose2d[] currentPathPoints;
    }
    
    public default void updateInputs(ZoroIOInputs inputs) {}

}
