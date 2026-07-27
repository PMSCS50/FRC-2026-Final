package frc.robot.util.pathfinding.zoro;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Static class for interacting with the chosen pathfinding implementation
 */

public class ShinPathfinding {
  private static ShinPathfinder pathfinder = null;

  /**
   * Set the pathfinder that should be used by the path following commands
   *
   * @param pathfinder The pathfinder to use
   */
  public static void setPathfinder(ShinPathfinder pathfinder) {
    ShinPathfinding.pathfinder = pathfinder;
  }

  /** Ensure that a pathfinding implementation has been chosen. If not, set it to the default. */
  public static void ensureInitialized() {
    if (pathfinder == null) {
      // Hasn't been initialized yet, use the default implementation

      pathfinder = new RoronoaZoro();
    }
  }

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  public static boolean isNewPathAvailable() {
    return pathfinder.isNewPathAvailable();
  }

  /**
   * Get the most recently calculated path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  public static PathPlannerPath getCurrentPath(
      PathConstraints constraints, GoalEndState goalEndState) {
    return pathfinder.getCurrentPath(constraints, goalEndState);
  }

  /**
   * Set the start position to pathfind from
   *
   * @param startPosition Start position on the field. If this is within an obstacle it will be
   *     moved to the nearest non-obstacle node.
   */
  public static void setStartPosition(Translation2d startPosition) {
    pathfinder.setStartPosition(startPosition);
  }

  /**
   * Set the start rotation
   *
   * @param startRotation Start rotation on the field.
   */
  public static void setStartRotation(Rotation2d startRotation) {
    pathfinder.setStartRotation(startRotation);
  }

  /**
   * Set the stop positions for multistop pathing
   *
   * @param stopPoses Start position on the field. If this is within an obstacle it will be
   *     moved to the nearest non-obstacle node.
   */
  public static void setStops(List<Pose2d> stopPoses) {
    pathfinder.setStops(stopPoses);
  }

  /**
   * Set the goal position to pathfind to
   *
   * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
   *     to the nearest non-obstacle node.
   */
  public static void setGoalPosition(Translation2d goalPosition) {
    pathfinder.setGoalPosition(goalPosition);
  }

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
   *     opposite corners of a bounding box.
   * @param currentRobotPos The current position of the robot. This is needed to change the start
   *     position of the path if the robot is now within an obstacle.
   */
  public static void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    pathfinder.setDynamicObstacles(obs, currentRobotPos);
  }
}