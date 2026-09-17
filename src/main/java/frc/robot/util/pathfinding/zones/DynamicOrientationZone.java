package frc.robot.util.pathfinding.zones;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;

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

    @Override
    public List<RotationTarget> createRotationTargets(double entry, double exit) {
        ArrayList<RotationTarget> rotationTargets = new ArrayList<>();
        //Needs the entry and exit index locations inside the PathPoints() array.
        for (PathPoint point : ZoneUtil.getPointsBetween(entry, exit)) {
            Rotation2d rotation =
                target.getTranslation()
                    .minus(point.position)
                    .getAngle()
                    .plus(offsetSupplier.get());

            rotationTargets.add(
                new RotationTarget(
                    point.waypointRelativePos,
                    rotation
                )
            );
        }
        return rotationTargets;
    }

    @Override
    public List<PointTowardsZone> createPointTowardsZones(double entry, double exit) { return new ArrayList<>();}

    @Override
    public List<ConstraintsZone> createConstraintsZones(double entry, double exit) { return new ArrayList<>();}

    @Override
    public List<EventMarker> createEventMarkers(double entry, double exit) { return new ArrayList<>();}

}