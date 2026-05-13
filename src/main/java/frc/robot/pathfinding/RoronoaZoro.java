package frc.robot.pathfinding;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.LocalADStar;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Custom pathfinder extending AD* with support for many different Zones
 * 
 * Zones are areas on the field that trigger certain behaviors when the robot is inside them.
 * 
 * Zones can be toggled active/inactive at runtime.
 * 
 * There are 4 types of zones:
 * 1. RotationZone: When the robot enters, it starts rotating towards a specified angle
 * 2. OrientationZone: When the robot enters, it starts orienting towards a specified target
 * 3. ConstraintZone: When the robot enters, certain path constraints are applied
 * 4. EventZone: When the robot enters, a specified command is triggered
 * 
 * Although one can already populate path files with these in Pathplanner,
 * it was not possible for pathfinding, which is why I created this extension.
 * 
 * Furthermore, the class takes your current velocity and rotation as its ideal starting state.
 * 
 */

public class RoronoaZoro extends LocalADStar {

    public RoronoaZoro() {
        super();
    }

    private Supplier<IdealStartingState> startingState;

    //Links Pathmaster method to RoronoaZoro.
    public void setStartingStateSupplier(Supplier<IdealStartingState> supplier) {
        this.startingState = supplier;
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        //Gets LocalAD* path to fill in zones
        PathPlannerPath basePath = super.getCurrentPath(constraints, goalEndState);
        if (basePath == null) return null;

        //Path waypoints
        List<Waypoint> waypoints = basePath.getWaypoints();
        if (waypoints.size() < 2) return null;

        //Loop through all pathpoints. If it enters or exits a zone, get waypoint relative pose
        List<PathPoint> points = basePath.getAllPathPoints();
        List<PathZone> activeZones = ZoneManager.getActiveZones();

        List<RotationTarget> rotationTargets = new ArrayList<>();
        List<PointTowardsZone> pointTowardsZones = new ArrayList<>();
        List<ConstraintsZone> constraintZones = new ArrayList<>();
        List<EventMarker> eventMarkers = new ArrayList<>();

        for (PathZone zone : activeZones) {

            int entryIndex = -1;
            int exitIndex  = -1;

            for (int i = 0; i < points.size(); i++) {
                if (zone.containsPoint(points.get(i).position())) {
                    if (entryIndex < 0) entryIndex = i;
                    exitIndex = i;
                }
            }

            if (entryIndex < 0) continue;

            double entryWaypointIndex = points.get(entryIndex).waypointRelativePos();
            double exitWaypointIndex  = points.get(exitIndex).waypointRelativePos();

            if (zone instanceof OrientationZone oz) {
                //Turns OZ into a PointTowardsZone
                pointTowardsZones.add(new PointTowardsZone(
                    zone.name, 
                    oz.getTarget().getTranslation(), 
                    entryWaypointIndex, 
                    exitWaypointIndex));
            
            } else if (zone instanceof RotationZone rz) {
                //Turns RZ into 2 rotation targets to force fixed heading throughout
                rotationTargets.add(new RotationTarget(
                    entryWaypointIndex, 
                    rz.getRotation()));

                rotationTargets.add(new RotationTarget(
                    exitWaypointIndex,   rz.getRotation()));

            } else if (zone instanceof ConstraintZone cz) {
                //Turns CZ into a ConstraintsZone
                constraintZones.add(new ConstraintsZone(
                    entryWaypointIndex,
                    exitWaypointIndex,
                    cz.getConstraints()));

            } else if (zone instanceof EventZone ez) {
                //Turns EZ into an EventMarker
                eventMarkers.add(new EventMarker(
                    zone.name, 
                    entryWaypointIndex, 
                    exitWaypointIndex, 
                    ez.getEvent()));
            }
        }

        return new PathPlannerPath(
            waypoints,
            rotationTargets,                                        //rotation zones
            pointTowardsZones,                                      //orientation zones
            constraintZones,                                        //constraint zones
            eventMarkers,                                           //event zones
            constraints,                                            //path constraints
            (startingState != null) ? startingState.get() : null,   //current velocity + heading
            goalEndState,                                           //goal end state
            false                                                   //never flip for red alliance
        );
    }
}