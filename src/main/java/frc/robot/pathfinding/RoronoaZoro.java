package frc.robot.pathfinding;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.LocalADStar;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

/**
 * Custom pathfinder extending AD* with support for:
 * RotationZone: robot holds a fixed heading through a field region
 * OrientationZone: robot continuously faces a target pose (via PointTowardsZone)
 * Zones can be toggled active/inactive at runtime.
 */
public class RoronoaZoro extends LocalADStar {

    private final HashMap<PathZone, Boolean> allZones = new HashMap<>();

    public RoronoaZoro() {
        super();
    }

    //Zone management. There are two types of zones: RotationZones and OrientationZones, both extending Pathzone.
    void addZone(PathZone zone, boolean active) {
        allZones.put(zone, active);
    }

    void setZoneState(String zoneName, boolean newState) {
        for (Map.Entry<PathZone, Boolean> entry : allZones.entrySet()) {
            if (entry.getKey().name.equals(zoneName)) {
                entry.setValue(newState);
                return;
            }
        }
        DriverStation.reportWarning(
            "[RoronoaZoro] Zone '" + zoneName + "' not found", false);
    }

    void setAllZones(boolean newState) {
        for (Map.Entry<PathZone, Boolean> entry : allZones.entrySet()) {
            entry.setValue(newState);
        }
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {

        PathPlannerPath basePath = super.getCurrentPath(constraints, goalEndState);
        if (basePath == null) return null;

        List<Waypoint> waypoints = basePath.getWaypoints();
        if (waypoints.size() < 2) return null;

        // Filter to active zones only
        List<PathZone> activeZones = allZones.entrySet().stream()
            .filter(Map.Entry::getValue)
            .map(Map.Entry::getKey)
            .collect(Collectors.toList());

        List<RotationTarget> rotationTargets     = new ArrayList<>();
        List<PointTowardsZone> pointTowardsZones = new ArrayList<>();

        // Precompute cumulative distances along path waypoints
        List<Translation2d> anchors = waypoints.stream()
            .map(Waypoint::anchor)
            .collect(Collectors.toList());

        double[] cumulativeDist = new double[anchors.size()];
        cumulativeDist[0] = 0.0;
        for (int i = 1; i < anchors.size(); i++) {
            cumulativeDist[i] = cumulativeDist[i - 1]
                + anchors.get(i).getDistance(anchors.get(i - 1));
        }
        double totalPathLength = cumulativeDist[anchors.size() - 1];

        for (PathZone zone : activeZones) {

            double entryFraction = -1;
            double exitFraction  = -1;

            for (int s = 0; s <= 500; s++) {
                double t = (double) s / 500;
                Translation2d point = interpolateAlongPath(
                    anchors, cumulativeDist, t * totalPathLength);
                if (zone.containsPoint(point)) {
                    if (entryFraction < 0) entryFraction = t;
                    exitFraction = t;
                }
            }

            if (entryFraction < 0) continue;

            double leadIn = 0.3;
            double leadInFraction = Math.max(0,
                (entryFraction * totalPathLength - leadIn) / totalPathLength);

            double leadInIndex = fractionToWaypointIndex(
                leadInFraction, totalPathLength, cumulativeDist);
            double exitIndex = fractionToWaypointIndex(
                exitFraction, totalPathLength, cumulativeDist);

            if (zone instanceof OrientationZone orientationZone) {
                pointTowardsZones.add(new PointTowardsZone(
                    zone.name,
                    orientationZone.getTarget().getTranslation(),
                    leadInIndex,
                    exitIndex
                ));

            } else if (zone instanceof RotationZone rotationZone) {
                rotationTargets.add(new RotationTarget(
                    leadInIndex, rotationZone.getRotation()));
                rotationTargets.add(new RotationTarget(
                    exitIndex, rotationZone.getRotation()));
            }

            DriverStation.reportWarning(
                "[RoronoaZoro] Zone '" + zone.name + "' active" +
                " entry=" + String.format("%.2f", entryFraction) +
                " exit="  + String.format("%.2f", exitFraction), false);
        }

        return new PathPlannerPath(
            waypoints,
            rotationTargets,
            pointTowardsZones,
            List.of(),
            List.of(),
            constraints,
            null,
            goalEndState,
            false
        );
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    private Translation2d interpolateAlongPath(
            List<Translation2d> anchors,
            double[] cumulativeDist,
            double distAlongPath) {

        for (int i = 1; i < anchors.size(); i++) {
            if (cumulativeDist[i] >= distAlongPath) {
                double segmentLength = cumulativeDist[i] - cumulativeDist[i - 1];
                if (segmentLength < 1e-6) return anchors.get(i);
                double t = (distAlongPath - cumulativeDist[i - 1]) / segmentLength;
                return anchors.get(i - 1).interpolate(anchors.get(i), t);
            }
        }
        return anchors.get(anchors.size() - 1);
    }

    private double fractionToWaypointIndex(
            double pathFraction,
            double totalPathLength,
            double[] cumulativeDist) {

        double targetDist = pathFraction * totalPathLength;
        for (int i = 1; i < cumulativeDist.length; i++) {
            if (cumulativeDist[i] >= targetDist) {
                double segmentLength = cumulativeDist[i] - cumulativeDist[i - 1];
                if (segmentLength < 1e-6) return i;
                double t = (targetDist - cumulativeDist[i - 1]) / segmentLength;
                return (i - 1) + t;
            }
        }
        return cumulativeDist.length - 1;
    }
}