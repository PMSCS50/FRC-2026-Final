package frc.robot.pathfinding;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A zone where the robot holds a fixed chassis heading throughout.
 * Implemented via RotationTargets at zone lead-in and exit.
 * Example: always face forward through the trench to protect the intake.
 */
public class RotationZone extends PathZone {

    private final Rotation2d rotation;

    public RotationZone(
            String name,
            Translation2d min,
            Translation2d max,
            Rotation2d rotation) {
        super(name, min, max);
        this.rotation = rotation;
    }

    public Rotation2d getRotation() {
        return rotation;
    }
}