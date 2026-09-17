package frc.robot.util.pathfinding.zones;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * // *A zone where the robot holds a fixed chassis heading throughout.
 * // ?Application (REBUILT): always face forward through the trench to not slam the intake into a wall.
 */
public class RotationZone extends PathZone {

    private final Rotation2d rotation;

    public RotationZone(String name, Translation2d min, Translation2d max, Rotation2d rotation) {
        super(name, min, max);
        this.rotation = rotation;
    }

    public Rotation2d getRotation() {
        return rotation;
    }

    @Override
    public List<RotationTarget> createRotationTargets(double entry, double exit) { 
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