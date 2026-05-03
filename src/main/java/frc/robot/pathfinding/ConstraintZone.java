package frc.robot.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A zone where the robot changes its path constraints
 * Example: forcing the robot to slow down in a certain area, or to turn faster in another area
 */
public class ConstraintZone extends PathZone {

    private final PathConstraints constraints;

    public OrientationZone(String name,Translation2d min,Translation2d max, PathConstraints constraints) {
        super(name, min, max);
        this.constraints = constraints;
    }

    public PathConstraints getConstraints() {
        return constraints;
    }

}