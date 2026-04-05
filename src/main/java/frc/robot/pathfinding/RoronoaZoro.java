package frc.robot.pathfinding;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.LocalADStar;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

import frc.robot.pathfinding.OrientationZone;
import frc.robot.pathfinding.RotationZone;

//Custom Path finder that extends AD*.
//Should also incorporate rotation and alignment zones
//HEAVY WORK IN PROGRESS.
public class RoronoaZoro extends LocalADStar {

  public RoronoaZoro() {
    super();
  }
  
  private final List<PathZone> rotationZones = new ArrayList<>();

  public void addZone(PathZone zone) {
      rotationZones.add(zone);
  }

  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    for (PathZone zone : rotationZones) {
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

        double entryFraction = -1;
        double exitFraction  = -1;

        int sampleCount = 500;
        for (int s = 0; s <= sampleCount; s++) {
            double t = (double) s / sampleCount;
            double distAlongPath = t * totalPathLength;
            Translation2d point = interpolateAlongPath(
                anchors, cumulativeDist, distAlongPath
            );
            if (zone.containsPoint(point)) {
                if (entryFraction < 0) entryFraction = t;
                exitFraction = t;
            }
        }

        if (entryFraction < 0) continue;

        // Lead-in target — use rotation at zone entry point
        double leadIn = 0.3;
        double leadInFraction = Math.max(0,
            (entryFraction * totalPathLength - leadIn) / totalPathLength
        );
        double leadInIndex = fractionToWaypointIndex(
            leadInFraction, totalPathLength, cumulativeDist
        );
        Translation2d entryPoint = interpolateAlongPath(
            anchors, cumulativeDist, entryFraction * totalPathLength
        );
        rotationTargets.add(new RotationTarget(leadInIndex,
            zone.getRotationAt(entryPoint)));

        // For OrientationZone — sample through the zone for smooth facing
        // For RotationZone — getRotationAt always returns the same angle
        // so extra samples are harmless but unnecessary; add only entry+exit
        boolean isOrientation = zone instanceof OrientationZone;
        int zoneSamples = isOrientation ? 20 : 1;

        for (int s = 0; s <= zoneSamples; s++) {
            double t = entryFraction + 
                (double) s / zoneSamples * (exitFraction - entryFraction);
            double dist = t * totalPathLength;
            Translation2d point = interpolateAlongPath(
                anchors, cumulativeDist, dist
            );
            double waypointIndex = fractionToWaypointIndex(
                t, totalPathLength, cumulativeDist
            );
            rotationTargets.add(new RotationTarget(
                waypointIndex,
                zone.getRotationAt(point)
            ));
        }

        // Exit target — ensures rotation holds exactly to zone boundary
        double exitIndex = fractionToWaypointIndex(
            exitFraction, totalPathLength, cumulativeDist
        );
        Translation2d exitPoint = interpolateAlongPath(
            anchors, cumulativeDist, exitFraction * totalPathLength
        );
        rotationTargets.add(new RotationTarget(exitIndex,
            zone.getRotationAt(exitPoint)));
    }

        // Rebuild with rotation targets injected
        return new PathPlannerPath(
            waypoints,
            rotationTargets,
            List.of(),   // no constraint zones
            List.of(),   // no point toward zones
            List.of(),   // no event markers
            constraints,
            null,        // no ideal starting state for on-the-fly
            goalEndState,
            false
        );
    }

    private int getNearestWaypointIndex(
            List<Waypoint> waypoints, Translation2d target) {
        int nearest = 0;
        double minDist = Double.MAX_VALUE;
        for (int i = 0; i < waypoints.size(); i++) {
            double dist = waypoints.get(i).anchor().getDistance(target);
            if (dist < minDist) {
                minDist = dist;
                nearest = i;
            }
        }
        return nearest;
    }
}