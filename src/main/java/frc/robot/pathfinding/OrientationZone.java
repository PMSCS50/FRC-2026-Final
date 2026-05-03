package frc.robot.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A zone where the robot continuously faces toward a target field pose,

 * Example: always face the hub while on the alliance zone
 */
public class OrientationZone extends PathZone {

    private final Pose2d target;

    public OrientationZone(String name,Translation2d min,Translation2d max, Pose2d target) {
        super(name, min, max);
        this.target = target;
    }

    public Pose2d getTarget() {
        return target;
    }

}