package frc.robot.util.pathfinding.zones;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 // *A zone where the robot continuously faces toward a target field pose, but at a changing offset value
 // ?Application (REBUILT): Facing the hub while shooting. Dynamic offset accounts for a moving robot
 */
public class DynamicOrientationZone extends PathZone {

    private final Pose2d target;
    private final Supplier<Rotation2d> offsetSupplier;

    public DynamicOrientationZone(String name,Translation2d min,Translation2d max, Pose2d target, Supplier<Rotation2d> offsetSupplier) {
        super(name, min, max);
        this.target = target;
        this.offsetSupplier = offsetSupplier;
    }

    public Pose2d getTarget() {
        return target;
    }

    public Supplier<Rotation2d> getOffset() {
        return offsetSupplier;
    }

}