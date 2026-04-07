package frc.robot.pathfinding;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * AdvantageKit-compatible wrapper around RoronoaZoro.
 */
public class RoronoaZoroAK implements Pathfinder {

    private final ZoroIO io = new ZoroIO();

    public RoronoaZoroAK() {
        // RoronoaZoro instantiated inside ZoroIO
    }

    // -----------------------------------------------------------------------
    // Zone delegation
    // -----------------------------------------------------------------------

    public void addZone(PathZone zone, boolean active) {
        io.zoro.addZone(zone, active);
    }

    public void setZoneState(String zoneName, boolean newState) {
        io.zoro.setZoneState(zoneName, newState);
    }

    public void setAllZones(boolean newState) {
        io.zoro.setAllZones(newState);
    }

    // -----------------------------------------------------------------------
    // Pathfinder interface
    // -----------------------------------------------------------------------

    @Override
    public boolean isNewPathAvailable() {
        if (!Logger.hasReplaySource()) {
            io.updateIsNewPathAvailable();
        }
        Logger.processInputs("RoronoaZoroAK", io);
        return io.isNewPathAvailable;
    }

    @Override
    public PathPlannerPath getCurrentPath(
            PathConstraints constraints, GoalEndState goalEndState) {
        if (!Logger.hasReplaySource()) {
            io.updateCurrentPathPoints(constraints, goalEndState);
        }
        Logger.processInputs("RoronoaZoroAK", io);

        if (io.currentPathPoints.isEmpty()) return null;

        return PathPlannerPath.fromPathPoints(
            io.currentPathPoints, constraints, goalEndState);
    }

    @Override
    public void setStartPosition(Translation2d startPosition) {
        if (!Logger.hasReplaySource()) {
            io.zoro.setStartPosition(startPosition);
        }
    }

    @Override
    public void setGoalPosition(Translation2d goalPosition) {
        if (!Logger.hasReplaySource()) {
            io.zoro.setGoalPosition(goalPosition);
        }
    }

    @Override
    public void setDynamicObstacles(
            List<Pair<Translation2d, Translation2d>> obs,
            Translation2d currentRobotPos) {
        if (!Logger.hasReplaySource()) {
            io.zoro.setDynamicObstacles(obs, currentRobotPos);
        }
    }

    // -----------------------------------------------------------------------
    // AK IO layer
    // -----------------------------------------------------------------------

    private static class ZoroIO implements LoggableInputs {

        public final RoronoaZoro zoro = new RoronoaZoro();

        public boolean isNewPathAvailable = false;
        public List<PathPoint> currentPathPoints = Collections.emptyList();

        @Override
        public void toLog(LogTable table) {
            table.put("IsNewPathAvailable", isNewPathAvailable);

            double[] pointsLogged = new double[currentPathPoints.size() * 2];
            int idx = 0;
            for (PathPoint point : currentPathPoints) {
                pointsLogged[idx]     = point.position.getX();
                pointsLogged[idx + 1] = point.position.getY();
                idx += 2;
            }
            table.put("CurrentPathPoints", pointsLogged);
        }

        @Override
        public void fromLog(LogTable table) {
            isNewPathAvailable = table.get("IsNewPathAvailable", false);

            double[] pointsLogged = table.get("CurrentPathPoints", new double[0]);
            List<PathPoint> pathPoints = new ArrayList<>();
            for (int i = 0; i < pointsLogged.length; i += 2) {
                pathPoints.add(new PathPoint(
                    new Translation2d(pointsLogged[i], pointsLogged[i + 1]),
                    null
                ));
            }
            currentPathPoints = pathPoints;
        }

        public void updateIsNewPathAvailable() {
            isNewPathAvailable = zoro.isNewPathAvailable();
        }

        public void updateCurrentPathPoints(
                PathConstraints constraints, GoalEndState goalEndState) {
            PathPlannerPath currentPath = zoro.getCurrentPath(constraints, goalEndState);
            if (currentPath != null) {
                currentPathPoints = currentPath.getAllPathPoints();
            } else {
                currentPathPoints = Collections.emptyList();
            }
        }
    }
}