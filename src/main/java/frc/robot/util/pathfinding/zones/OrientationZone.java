package frc.robot.util.pathfinding.zones;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;

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

    @Override
    public List<RotationTarget> createRotationTargets(double entry, double exit) { return new ArrayList<>();}

    @Override
    public List<PointTowardsZone> createPointTowardsZones(double entry, double exit) { 
        PointTowardsZone ptz = new PointTowardsZone(name, target.getTranslation(), offset, entry, exit);
        return List.of(ptz);
    }

    @Override
    public List<ConstraintsZone> createConstraintsZones(double entry, double exit) { return new ArrayList<>();}

    @Override
    public List<EventMarker> createEventMarkers(double entry, double exit) { return new ArrayList<>();}

}