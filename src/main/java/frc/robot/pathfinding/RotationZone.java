package frc.robot.pathfinding;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A zone where the robot holds a fixed heading throughout.
 * A good example in REBUILT is always staying forward when going through the trench so our intake doesnt slam into the wall.
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

    @Override
    public Rotation2d getRotationAt(Translation2d pointOnPath) {
        return rotation;
    }
}