package frc.robot.util.pathfinding.zoro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

//Logging the minimal amount of stuff for us to perfectly recreate the original PathPlannerPath taken during log replay
//This makes sure that when we are replaying a log file, the bot will take the same paths from the match
public interface ZoroIO {

    @AutoLog
    public static class ZoroIOInputs {
        // Waypoints
        public int numWaypoints;
        public Pose2d[] waypointAnchors;
        public Pose2d[] waypointPrevControls;
        public Pose2d[] waypointNextControls;

        // Rotation targets
        public double[] rotationTargetPositions;
        public Rotation2d[] rotationTargetRotations;

        // Point towards zones
        public String[] ptZoneNames;
        public Translation2d[] ptZoneTargetPositions;
        public double[] ptZoneRotOffsets;
        public double[] ptZoneMinPositions;
        public double[] ptZoneMaxPositions;

        // Constraint zones
        public double[] constraintZoneMinPositions;
        public double[] constraintZoneMaxPositions;
        public double[] constraintZoneMaxVel;
        public double[] constraintZoneMaxAngVel;
        public double[] constraintZoneMaxAcc;
        public double[] constraintZoneMaxAngAcc;

        // Event markers
        public String[] eventTriggerNames;
        public double[] eventPositions;
        public double[] eventEndPositions;

        // Keep the isNewPathAvailable flag
        public boolean isNewPathAvailable;
    }
    
}
