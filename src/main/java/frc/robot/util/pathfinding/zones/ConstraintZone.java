package frc.robot.util.pathfinding.zones;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * *A zone where the robot changes its path constraints
 // ?Application (REBUILT): forcing the robot to slow down on the bump to avoid tipping over.
 */
public class ConstraintZone extends PathZone {

    private final PathConstraints constraints;

    public ConstraintZone(String name,Translation2d min,Translation2d max, PathConstraints constraints) {
        super(name, min, max);
        this.constraints = constraints;
    }

    public PathConstraints getConstraints() {
        return constraints;
    }

    @Override
    public List<RotationTarget> createRotationTargets(double entry, double exit) { return new ArrayList<>();}

    @Override
    public List<PointTowardsZone> createPointTowardsZones(double entry, double exit) { return new ArrayList<>();}

    @Override
    public List<ConstraintsZone> createConstraintsZones(double entry, double exit) { 
        ConstraintsZone cz = new ConstraintsZone(entry, exit, constraints);

        return new ArrayList<>(List.of(cz));
    }

    @Override
    public List<EventMarker> createEventMarkers(double entry, double exit) { return new ArrayList<>();}

}