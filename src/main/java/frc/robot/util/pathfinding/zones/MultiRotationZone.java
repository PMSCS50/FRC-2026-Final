package frc.robot.util.pathfinding.zones;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * // *A zone where the robot holds a fixed chassis heading throughout, and chooses the rotation that requires the least turning
 * // ?Example (REBUILT): always face forward through the trench to not slam the intake into a wall, 
 * // ?but to also avoid turning 180 degrees into the trench, risking hitting against the wall too
 */
public class MultiRotationZone extends PathZone {

    private final Rotation2d[] rotations;

    public MultiRotationZone(String name, Translation2d min, Translation2d max, Rotation2d... rotations) {
        super(name, min, max);
        this.rotations = rotations;
    }

    public Rotation2d getClosestRotation(Rotation2d lastRotation) {

        double minDifference = Double.MAX_VALUE;
        Rotation2d bestRotation = Rotation2d.kZero;

        for (Rotation2d rotation : rotations) {
            double diff = Math.abs(MathUtil.angleModulus(rotation.minus(lastRotation).getRadians()));

            if (diff < minDifference) {
                minDifference = diff;
                bestRotation = rotation;
            }
        }

        return bestRotation;
    }
}