package frc.robot.util.pathfinding;

// import edu.wpi.first.math.geometry.Pose2d;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * *A zone where the robot changes its path constraints
 * ?Example (REBUILT): forcing the robot to slow down on the bump to avoid tipping over.
 */
public class ConstraintZone extends PathZone {

    private final PathConstraints constraints;

    public ConstraintZone(String name,Translation2d min,Translation2d max, PathConstraints constraints) {
        super(name, min, max);
        this.constraints = constraints;
    }

    public PathConstraints getConstraints() {
        return constraints;
    }

}