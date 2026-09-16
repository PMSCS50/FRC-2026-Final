package frc.robot.util.pathfinding.zones;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 // *A zone where the robot continuously faces toward a target field pose,
 // ?Application (REBUILT): always face the hub while on the alliance zone
 */
public class OrientationZone extends PathZone {

    private final Pose2d target;
    private final Rotation2d offset;

    public OrientationZone(String name,Translation2d min,Translation2d max, Pose2d target) {
        super(name, min, max);
        this.target = target;
        this.offset = Rotation2d.kZero;
    }

    //Constructor with offset.
    public OrientationZone(String name,Translation2d min,Translation2d max, Pose2d target, Rotation2d offset) {
        super(name, min, max);
        this.target = target;
        this.offset = offset;
    }

    public Pose2d getTarget() {
        return target;
    }

    public Rotation2d getOffset() {
        return offset;
    }

}