package frc.robot.util.pathfinding.zoro;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class RoronoaZoroAK implements Pathfinder {

    private final RoronoaZoro zoro = new RoronoaZoro();
    private final ZoroIOInputsAutoLogged inputs = new ZoroIOInputsAutoLogged();
    private final String logKey = "RoronoaZoroAK";

    private List<EventMarker> currentPathEvents = new ArrayList<>();

    @Override
    public boolean isNewPathAvailable() {
        if (!Logger.hasReplaySource()) {
            inputs.isNewPathAvailable = zoro.isNewPathAvailable();
        }
        Logger.processInputs(logKey, inputs);
        return inputs.isNewPathAvailable;
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        if (!Logger.hasReplaySource()) {
            PathPlannerPath currentPath = zoro.getCurrentPath(constraints, goalEndState);
            
            if (currentPath != null) {
                List<PathPoint> points = currentPath.getAllPathPoints();
                currentPathEvents = currentPath.getEventMarkers();
                int size = points.size();

                Pose2d[] pathPoints = new Pose2d[size];
                Rotation2d[] rotationTargets = new Rotation2d[size];
                boolean[] hasRotationTarget = new boolean[size];
                double[] waypointRelativePoses = new double[size];
                
                // Flatten constraints into a 1D array: [maxV, maxAngV, maxA, maxAngA] per point
                double[] serializedConstraints = new double[size * 4];

                for (int i = 0; i < size; i++) {
                    PathPoint p = points.get(i);
                    pathPoints[i] = new Pose2d(p.position, new Rotation2d());
                    waypointRelativePoses[i] = p.waypointRelativePos;

                    // Check for null rotation targets to prevent NullPointerException
                    if (p.rotationTarget != null) {
                        hasRotationTarget[i] = true;
                        rotationTargets[i] = p.rotationTarget.rotation();
                    } else {
                        hasRotationTarget[i] = false;
                        rotationTargets[i] = new Rotation2d();
                    }

                    if (p.constraints != null) {
                        serializedConstraints[i * 4]     = p.constraints.maxVelocityMPS();
                        serializedConstraints[i * 4 + 1] = p.constraints.maxAngularVelocityRadPerSec();
                        serializedConstraints[i * 4 + 2] = p.constraints.maxAccelerationMPSSq();
                        serializedConstraints[i * 4 + 3] = p.constraints.maxAngularAccelerationRadPerSecSq();
                    }
                }

                inputs.currentPathPoints = pathPoints;
                inputs.currentRotationTargets = rotationTargets;
                inputs.hasRotationTarget = hasRotationTarget;
                inputs.waypointRelativePoses = waypointRelativePoses;
                inputs.allConstraints = serializedConstraints;
            } else {
                // Clear inputs if path is null to prevent stale path execution
                inputs.currentPathPoints = new Pose2d[0];
                inputs.currentRotationTargets = new Rotation2d[0];
                inputs.hasRotationTarget = new boolean[0];
                inputs.waypointRelativePoses = new double[0];
                inputs.allConstraints = new double[0];
            }
        }

        // On replay, processInputs populates 'inputs' from log file
        Logger.processInputs(logKey, inputs);

        if (inputs.currentPathPoints == null || inputs.currentPathPoints.length == 0) {
            return null;
        }

        // Reconstruct PathPlannerPath cleanly for live and replay modes
        List<PathPoint> reconstructedPoints = new ArrayList<>();
        for (int i = 0; i < inputs.currentPathPoints.length; i++) {
            RotationTarget rotTarget = inputs.hasRotationTarget[i]
                ? new RotationTarget(inputs.waypointRelativePoses[i], inputs.currentRotationTargets[i])
                : null;

            PathConstraints ptConstraints = new PathConstraints(
                inputs.allConstraints[i * 4],
                inputs.allConstraints[i * 4 + 1],
                inputs.allConstraints[i * 4 + 2],
                inputs.allConstraints[i * 4 + 3]
            );

            reconstructedPoints.add(
                new PathPoint(
                    inputs.currentPathPoints[i].getTranslation(),
                    rotTarget,
                    ptConstraints
                )
            );
        }

        PathPlannerPath finalPath = PathPlannerPath.fromPathPoints(reconstructedPoints, constraints, goalEndState);
        finalPath.getEventMarkers().addAll(currentPathEvents);

        return finalPath;
    }

    @Override
    public void setStartPosition(Translation2d startPosition) {
        if (!Logger.hasReplaySource()) {
            zoro.setStartPosition(startPosition);
        }
    }

    public void setStops(List<Pose2d> stops) {
        if (!Logger.hasReplaySource()) {
            zoro.setStops(stops);
        }
    }

    @Override
    public void setGoalPosition(Translation2d goalPosition) {
        if (!Logger.hasReplaySource()) {
            zoro.setGoalPosition(goalPosition);
        }
    }

    @Override
    public void setDynamicObstacles(
            List<Pair<Translation2d, Translation2d>> obs,
            Translation2d currentRobotPos) {
        if (!Logger.hasReplaySource()) {
            zoro.setDynamicObstacles(obs, currentRobotPos);
        }
    }
}