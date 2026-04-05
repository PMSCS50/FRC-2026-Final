package frc.robot.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A zone where the robot continuously faces toward a target field pose,
 * implemented via PathPlanner's PointTowardsZone.
 * PathPlanner handles all continuous facing math internally —
 * no manual rotation target sampling needed.
 * Example: always face the hub while passing through a shooting corridor.
 */
public class OrientationZone extends PathZone {

    private final Pose2d target;

    public OrientationZone(
            String name,
            Translation2d min,
            Translation2d max,
            Pose2d target) {
        super(name, min, max);
        this.target = target;
    }

    public Pose2d getTarget() {
        return target;
    }

}