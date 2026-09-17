package frc.robot.util.pathfinding.zones;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * // *A zone where the robot holds a fixed chassis heading throughout, and chooses the rotation that requires the least turning
 * // ?Application (REBUILT): always face forward through the trench to not slam the intake into a wall, 
 * // ?but to also avoid turning 180 degrees into the trench, risking hitting against the wall too
 */
public class MultiRotationZone extends PathZone {

    private final List<Rotation2d> rotations;

    public MultiRotationZone(String name, Translation2d min, Translation2d max, List<Rotation2d> rotations) {
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

    @Override
    public List<RotationTarget> createRotationTargets(double entry, double exit) { 
        Rotation2d rotation = getClosestRotation(ZoneUtil.getPreviousRotation(entry));
        RotationTarget rt1 = new RotationTarget(entry, rotation);
        RotationTarget rt2 = new RotationTarget(entry, rotation);
        return List.of(rt1, rt2);
    }

    @Override
    public List<PointTowardsZone> createPointTowardsZones(double entry, double exit) { return new ArrayList<>();}

    @Override
    public List<ConstraintsZone> createConstraintsZones(double entry, double exit) { return new ArrayList<>();}

    @Override
    public List<EventMarker> createEventMarkers(double entry, double exit) { return new ArrayList<>();}
}