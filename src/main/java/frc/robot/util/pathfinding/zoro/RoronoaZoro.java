package frc.robot.util.pathfinding.zoro;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.pathfinding.zones.ConstraintZone;
import frc.robot.util.pathfinding.zones.EventZone;
import frc.robot.util.pathfinding.zones.OrientationZone;
import frc.robot.util.pathfinding.zones.PathZone;
import frc.robot.util.pathfinding.zones.RotationZone;
import frc.robot.util.pathfinding.zones.ZoneManager;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.*;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * Custom pathfinder extending AD* with support for many different Zones, as well as multistop pathfinding.
 * Zones are areas on the field that trigger certain behaviors when the robot is inside them.
 * 
 * Zones can be toggled active/inactive at runtime.
 * * There are 4 types of zones:
 * 1. RotationZone: When the robot enters this zone, it starts rotating towards a specified angle
 * 2. OrientationZone: When the robot enters this zone, it starts orienting towards a specified target
 * 3. ConstraintZone: When the robot enters this zone, certain path constraints are applied
 * 4. EventZone: When the robot enters this zone, a specified command is triggered
 * 
 * * LocalADStar couldn't create paths from a->b->c smoothly, so this pathfinder also handles multiple stops.
 * * Although I am much prouder of this than the zones, i cant really say much more
 */
public class RoronoaZoro implements Pathfinder {
  private static final double SMOOTHING_ANCHOR_PCT = 0.8;
  private static final double EPS = 2.5;

  private double fieldLength = 16.54;
  private double fieldWidth = 8.02;

  private double nodeSize = 0.2;

  private int nodesX = (int) Math.ceil(fieldLength / nodeSize);
  private int nodesY = (int) Math.ceil(fieldWidth / nodeSize);

  private final Set<GridPosition> staticObstacles = new HashSet<>();
  private final Set<GridPosition> dynamicObstacles = new HashSet<>();
  private final Set<GridPosition> requestObstacles = new HashSet<>();

  private GridPosition requestStart;
  private Translation2d requestRealStartPos;
  private GridPosition requestGoal;
  private Translation2d requestRealGoalPos;
  private final List<GridPosition> requestStops = new ArrayList<>();
  private final List<Translation2d> requestRealStopPoses = new ArrayList<>();
  private final HashMap<Translation2d, Rotation2d> requestRealStopRotations = new HashMap<>();

  // Track the primary active state chain for multi-segment pathing
  private final List<ADStarSegment> activeStates = new ArrayList<>();

  private final Thread planningThread;
  private boolean requestMinor = true;
  private boolean requestMajor = true;
  private boolean requestReset = true;
  private volatile boolean newPathAvailable = false;

  private final ReadWriteLock pathLock = new ReentrantReadWriteLock();
  private final ReadWriteLock requestLock = new ReentrantReadWriteLock();

  private List<Waypoint> currentWaypoints = new ArrayList<>();
  private List<GridPosition> currentPathFull = new ArrayList<>();

  private final List<RotationTarget> rotationTargets = new ArrayList<>();
  private final List<PointTowardsZone> pointTowardsZones = new ArrayList<>();
  private final List<ConstraintsZone> constraintZones = new ArrayList<>();
  private final List<EventMarker> eventMarkers = new ArrayList<>();

  /** Create a new pathfinder that runs AD* locally in a background thread */
  public RoronoaZoro() {
    planningThread = new Thread(this::runThread);

    requestStart = new GridPosition(0, 0);
    requestRealStartPos = Translation2d.kZero;
    requestGoal = new GridPosition(0, 0);
    requestRealGoalPos = Translation2d.kZero;
    requestStops.clear();
    requestRealStopPoses.clear();

    staticObstacles.clear();
    dynamicObstacles.clear();

    File navGridFile = new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json");
    if (navGridFile.exists()) {
      try (BufferedReader br = new BufferedReader(new FileReader(navGridFile))) {
        StringBuilder fileContentBuilder = new StringBuilder();
        String line;
        while ((line = br.readLine()) != null) {
          fileContentBuilder.append(line);
        }

        String fileContent = fileContentBuilder.toString();
        JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

        nodeSize = ((Number) json.get("nodeSizeMeters")).doubleValue();
        JSONArray grid = (JSONArray) json.get("grid");
        nodesY = grid.size();
        for (int row = 0; row < grid.size(); row++) {
          JSONArray rowArray = (JSONArray) grid.get(row);
          if (row == 0) {
            nodesX = rowArray.size();
          }
          for (int col = 0; col < rowArray.size(); col++) {
            boolean isObstacle = (boolean) rowArray.get(col);
            if (isObstacle) {
              staticObstacles.add(new GridPosition(col, row));
            }
          }
        }

        JSONObject fieldSize = (JSONObject) json.get("field_size");
        fieldLength = ((Number) fieldSize.get("x")).doubleValue();
        fieldWidth = ((Number) fieldSize.get("y")).doubleValue();
      } catch (Exception e) {
        // Do nothing, use defaults
      }
    }

    requestObstacles.clear();
    requestObstacles.addAll(staticObstacles);
    requestObstacles.addAll(dynamicObstacles);

    requestReset = true;
    requestMajor = true;
    requestMinor = true;

    newPathAvailable = false;

    planningThread.setDaemon(true);
    planningThread.setName("ADStar Planning Thread");
    planningThread.start();
  }

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  @Override
  public boolean isNewPathAvailable() {
    return newPathAvailable;
  }

  /**
   * Get the most recently calculated path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    List<Waypoint> waypoints;

    pathLock.readLock().lock();
    waypoints = new ArrayList<>(currentWaypoints);
    pathLock.readLock().unlock();

    newPathAvailable = false;

    if (waypoints.size() < 2) {
      // Not enough points. Something got borked somewhere
      return null;
    }

    return fillZones(new PathPlannerPath(waypoints, constraints, null, goalEndState));
  }

  /**
   * Set the start position to pathfind from
   *
   * @param startPosition Start position on the field. If this is within an obstacle it will be
   * moved to the nearest non-obstacle node.
   */
  @Override
  public void setStartPosition(Translation2d startPosition) {
    GridPosition startPos = findClosestNonObstacle(getGridPos(startPosition), requestObstacles);

    if (startPos != null && !startPos.equals(requestStart)) {
      requestLock.writeLock().lock();
      requestStart = startPos;
      requestRealStartPos = startPosition;

      // Clear previous intermediate stops to prevent memory leakage across paths
      requestStops.clear();
      requestRealStopPoses.clear();

      requestMinor = true;
      newPathAvailable = false;
      requestLock.writeLock().unlock();
    }
  }

  /**
   * Set custom intermediate stops for start -> stop1 -> stop2 -> goal pathfinding.
   * Call this after setStartPosition but before setGoalPosition.
   *
   * @param stops List of physical coordinate positions representing stops
   */
  public void setStops(List<Pose2d> stops) {
    requestLock.writeLock().lock();
    requestStops.clear();
    requestRealStopPoses.clear();


    for (Pose2d stop : stops) {
      GridPosition gridPos = findClosestNonObstacle(getGridPos(stop.getTranslation()), requestObstacles);
      if (gridPos != null) {
        requestStops.add(gridPos);
        requestRealStopPoses.add(stop.getTranslation());
        requestRealStopRotations.put(stop.getTranslation(), stop.getRotation());
      }
    }

    requestMinor = true;
    requestMajor = true;
    requestReset = true;
    newPathAvailable = false;
    requestLock.writeLock().unlock();
  }

  /**
   * Set the goal position to pathfind to
   *
   * @param goalPosition Goal position on the field. If this is within an obstacle it will be moved
   * to the nearest non-obstacle node.
   */
  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    GridPosition gridPos = findClosestNonObstacle(getGridPos(goalPosition), requestObstacles);

    if (gridPos != null) {
      requestLock.writeLock().lock();
      requestGoal = gridPos;
      requestRealGoalPos = goalPosition;

      requestMinor = true;
      requestMajor = true;
      requestReset = true;
      newPathAvailable = false;
      requestLock.writeLock().unlock();
    }
  }

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
   * opposite corners of a bounding box.
   * @param currentRobotPos The current position of the robot. This is needed to change the start
   * position of the path if the robot is now within an obstacle.
   */
  @Override
  public void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    Set<GridPosition> newObs = new HashSet<>();

    for (var obstacle : obs) {
      var gridPos1 = getGridPos(obstacle.getFirst());
      var gridPos2 = getGridPos(obstacle.getSecond());

      int minX = Math.min(gridPos1.x, gridPos2.x);
      int maxX = Math.max(gridPos1.x, gridPos2.x);

      int minY = Math.min(gridPos1.y, gridPos2.y);
      int maxY = Math.max(gridPos1.y, gridPos2.y);

      for (int x = minX; x <= maxX; x++) {
        for (int y = minY; y <= maxY; y++) {
          newObs.add(new GridPosition(x, y));
        }
      }
    }

    dynamicObstacles.clear();
    dynamicObstacles.addAll(newObs);
    requestLock.writeLock().lock();
    requestObstacles.clear();
    requestObstacles.addAll(staticObstacles);
    requestObstacles.addAll(dynamicObstacles);
    requestLock.writeLock().unlock();

    pathLock.readLock().lock();
    boolean recalculate = false;
    for (GridPosition pos : currentPathFull) {
      if (requestObstacles.contains(pos)) {
        recalculate = true;
        break;
      }
    }
    pathLock.readLock().unlock();

    if (recalculate) {
      requestLock.writeLock().lock();
      requestStart = findClosestNonObstacle(getGridPos(currentRobotPos), requestObstacles);
      requestRealStartPos = currentRobotPos;
      requestMinor = true;
      requestMajor = true;
      requestReset = true;
      newPathAvailable = false;
      requestLock.writeLock().unlock();
    }
  }

  @SuppressWarnings("BusyWait")
  private void runThread() {
    while (true) {
      try {
        requestLock.readLock().lock();
        boolean reset = requestReset;
        boolean minor = requestMinor;
        boolean major = requestMajor;
        GridPosition start = requestStart;
        Translation2d realStart = requestRealStartPos;
        GridPosition goal = requestGoal;
        Translation2d realGoal = requestRealGoalPos;
        List<GridPosition> stops = new ArrayList<>(requestStops);
        List<Translation2d> realStops = new ArrayList<>(requestRealStopPoses);
        Set<GridPosition> obstacles = new HashSet<>(requestObstacles);

        // Change the request booleans based on what will be done this loop
        if (reset) {
          requestReset = false;
        }

        if (minor) {
          requestMinor = false;
        } else if (major && activeStates.stream().allMatch(s -> (s.eps - 0.5) <= 1.0)) {
          requestMajor = false;
        }
        requestLock.readLock().unlock();

        if (reset || minor || major) {
          doWork(new PathRequest(reset, minor, major, start, stops, goal, realStart, realStops, realGoal, obstacles));
        } else {
          try {
            Thread.sleep(10);
          } catch (InterruptedException e) {
            throw new RuntimeException(e);
          }
        }
      } catch (Exception e) {
        // Something messed up. Reset and hope for the best
        requestLock.writeLock().lock();
        requestReset = true;
        requestLock.writeLock().unlock();
      }
    }
  }

  private void doWork(PathRequest pathReq) {
    // 1. Reconstruct the sequential path chain: start -> stops -> goal
    List<GridPosition> stops = new ArrayList<>(pathReq.sStops());
    stops.add(pathReq.sGoal());

    if (pathReq.needsReset()) {
      activeStates.clear();
      GridPosition currentStart = pathReq.sStart();
      for (GridPosition nextStop : stops) {
        ADStarSegment stateSegment = new ADStarSegment(currentStart, nextStop);
        reset(stateSegment);
        activeStates.add(stateSegment);
        currentStart = nextStop;
      }
    }

    if (pathReq.doMinor()) {
      List<GridPosition> pathPositions = new ArrayList<>();

      for (int i = 0; i < activeStates.size(); i++) {
        ADStarSegment state = activeStates.get(i);
        computeOrImprovePath(state, pathReq.obstacles());
        List<GridPosition> extractedPath = extractPath(state, pathReq.obstacles());
        
        // --- STRICT STOP ENFORCEMENT GUARD ---
        // If the path failed to reach the intended stop, force-append the target stop node 
        // to guarantee the trajectory sequence is perfectly preserved.
        if (extractedPath.isEmpty()) {
          extractedPath.add(state.start);
          if (!state.start.equals(state.goal)) {
            extractedPath.add(state.goal);
          }
        } else if (!extractedPath.get(extractedPath.size() - 1).equals(state.goal)) {
          extractedPath.add(state.goal);
        }

        // Clean stitching logic: strip overlapping duplicates between segments
        if (i > 0 && !extractedPath.isEmpty()) {
          extractedPath.remove(0);
        }
        pathPositions.addAll(extractedPath);
      }

      List<Waypoint> waypoints =
          createWaypoints(pathPositions, pathReq.realStartPos(), pathReq.sStops(), pathReq.realStopPoses(), pathReq.realGoalPos(), pathReq.obstacles());

      pathLock.writeLock().lock();
      currentPathFull = pathPositions;
      currentWaypoints = waypoints;
      pathLock.writeLock().unlock();

      newPathAvailable = true;
    } else if (pathReq.doMajor()) {
      boolean updatedAny = false;

      for (ADStarSegment state : activeStates) {
        if (state.eps > 1.0) {
          state.eps -= 0.5;
          state.open.putAll(state.incons);

          state.open.replaceAll((s, v) -> key(s, state));
          state.closed.clear();

          computeOrImprovePath(state, pathReq.obstacles());
          updatedAny = true;
        }
      }

      if (updatedAny) {
        List<GridPosition> pathPositions = new ArrayList<>();
        for (int i = 0; i < activeStates.size(); i++) {
          ADStarSegment state = activeStates.get(i);
          List<GridPosition> extractedPath = extractPath(state, pathReq.obstacles());
          
          // --- STRICT STOP ENFORCEMENT GUARD ---
          if (extractedPath.isEmpty()) {
            extractedPath.add(state.start);
            if (!state.start.equals(state.goal)) {
              extractedPath.add(state.goal);
            }
          } else if (!extractedPath.get(extractedPath.size() - 1).equals(state.goal)) {
            extractedPath.add(state.goal);
          }

          if (i > 0 && !extractedPath.isEmpty()) {
            extractedPath.remove(0);
          }
          pathPositions.addAll(extractedPath);
        }

        List<Waypoint> waypoints =
            createWaypoints(pathPositions, pathReq.realStartPos(), pathReq.sStops(), pathReq.realStopPoses(), pathReq.realGoalPos(), pathReq.obstacles());

        pathLock.writeLock().lock();
        currentPathFull = pathPositions;
        currentWaypoints = waypoints;
        pathLock.writeLock().unlock();

        newPathAvailable = true;
      }
    }
  }

  private List<GridPosition> extractPath(ADStarSegment state, Set<GridPosition> obstacles) {
    if (state.goal.equals(state.start)) {
      return new ArrayList<>();
    }

    List<GridPosition> path = new ArrayList<>();
    path.add(state.start);

    var s = state.start;
    int maxLimit = Math.max(1000, nodesX * nodesY);

    for (int k = 0; k < maxLimit; k++) {
      HashMap<GridPosition, Double> gList = new HashMap<>();

      for (GridPosition x : getOpenNeighbors(s, obstacles)) {
        gList.put(x, state.g.getOrDefault(x, Double.POSITIVE_INFINITY));
      }

      Map.Entry<GridPosition, Double> min = Map.entry(state.goal, Double.POSITIVE_INFINITY);
      for (var entry : gList.entrySet()) {
        if (entry.getValue() < min.getValue()) {
          min = entry;
        }
      }
      s = min.getKey();

      path.add(s);
      if (s.equals(state.goal)) {
        break;
      }
    }

    return path;
  }

  private List<Waypoint> createWaypoints(
      List<GridPosition> path,
      Translation2d realStartPos,
      List<GridPosition> stops,
      List<Translation2d> realStopPoses,
      Translation2d realGoalPos,
      Set<GridPosition> obstacles) {
    if (path.isEmpty()) {
      return new ArrayList<>();
    }

    List<GridPosition> simplifiedPath = new ArrayList<>();
    List<Integer> stopIndexes = new ArrayList<>();
    simplifiedPath.add(path.get(0));
    for (int i = 1; i < path.size() - 1; i++) {
      if (!walkable(simplifiedPath.get(simplifiedPath.size() - 1), path.get(i + 1), obstacles) || stops.contains(path.get(i))) {
        simplifiedPath.add(path.get(i));
        if (stops.contains(path.get(i))) {
          stopIndexes.add(simplifiedPath.size() - 1);
        }
      }
    }
    simplifiedPath.add(path.get(path.size() - 1));

    List<Translation2d> fieldPosPath = new ArrayList<>();
    for (GridPosition pos : simplifiedPath) {
      fieldPosPath.add(gridPosToTranslation2d(pos));
    }

    if (fieldPosPath.size() < 2) {
      return new ArrayList<>();
    }

    // Replace start and end positions with their real positions
    fieldPosPath.set(0, realStartPos);
    fieldPosPath.set(fieldPosPath.size() - 1, realGoalPos);

    for (int i = 0; i < stopIndexes.size(); i++) {
      fieldPosPath.set(stopIndexes.get(i), realStopPoses.get(i));
    }

    List<Pose2d> pathPoses = new ArrayList<>();
    pathPoses.add(
        new Pose2d(fieldPosPath.get(0), fieldPosPath.get(1).minus(fieldPosPath.get(0)).getAngle()));
    for (int i = 1; i < fieldPosPath.size() - 1; i++) {
      Translation2d last = fieldPosPath.get(i - 1);
      Translation2d current = fieldPosPath.get(i);
      Translation2d next = fieldPosPath.get(i + 1);

      Translation2d anchor1 = current.minus(last).times(SMOOTHING_ANCHOR_PCT).plus(last);
      Rotation2d heading1 = current.minus(last).getAngle();
      Translation2d anchor2 = current.minus(next).times(SMOOTHING_ANCHOR_PCT).plus(next);
      Rotation2d heading2 = next.minus(anchor2).getAngle();

      pathPoses.add(new Pose2d(anchor1, heading1));
      if (stopIndexes.contains(i)) {
        pathPoses.add(new Pose2d(current, heading1));
      }
      pathPoses.add(new Pose2d(anchor2, heading2));
    }
    pathPoses.add(
        new Pose2d(
            fieldPosPath.get(fieldPosPath.size() - 1),
            fieldPosPath
                .get(fieldPosPath.size() - 1)
                .minus(fieldPosPath.get(fieldPosPath.size() - 2))
                .getAngle()));

    return PathPlannerPath.waypointsFromPoses(pathPoses);
  }

  private GridPosition findClosestNonObstacle(GridPosition pos, Set<GridPosition> obstacles) {
    if (!obstacles.contains(pos)) {
      return pos;
    }

    Set<GridPosition> visited = new HashSet<>();
    Queue<GridPosition> queue = new LinkedList<>(getAllNeighbors(pos));

    while (!queue.isEmpty()) {
      GridPosition check = queue.poll();
      if (!obstacles.contains(check)) {
        return check;
      }
      visited.add(check);

      for (GridPosition neighbor : getAllNeighbors(check)) {
        if (!visited.contains(neighbor) && !queue.contains(neighbor)) {
          queue.add(neighbor);
        }
      }
    }
    return null;
  }

  private boolean walkable(GridPosition s1, GridPosition s2, Set<GridPosition> obstacles) {
    int x0 = s1.x;
    int y0 = s1.y;
    int x1 = s2.x;
    int y1 = s2.y;

    int dx = Math.abs(x1 - x0);
    int dy = Math.abs(y1 - y0);
    int x = x0;
    int y = y0;
    int n = 1 + dx + dy;
    int xInc = (x1 > x0) ? 1 : -1;
    int yInc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; n--) {
      if (obstacles.contains(new GridPosition(x, y))) {
        return false;
      }

      if (error > 0) {
        x += xInc;
        error -= dy;
      } else if (error < 0) {
        y += yInc;
        error += dx;
      } else {
        x += xInc;
        y += yInc;
        error -= dy;
        error += dx;
        n--;
      }
    }

    return true;
  }

  private void reset(ADStarSegment state) {
    state.g.clear();
    state.rhs.clear();
    state.open.clear();
    state.incons.clear();
    state.closed.clear();

    for (int x = 0; x < nodesX; x++) {
      for (int y = 0; y < nodesY; y++) {
        state.g.put(new GridPosition(x, y), Double.POSITIVE_INFINITY);
        state.rhs.put(new GridPosition(x, y), Double.POSITIVE_INFINITY);
      }
    }

    state.rhs.put(state.goal, 0.0);
    state.eps = EPS;

    state.open.put(state.goal, key(state.goal, state));
  }

  private void computeOrImprovePath(ADStarSegment state, Set<GridPosition> obstacles) {
    while (true) {
      var sv = topKey(state);
      if (sv == null) {
        break;
      }
      var s = sv.getFirst();
      var v = sv.getSecond();

      Pair<Double, Double> startKey = key(state.start, state);
      double startRhs = state.rhs.getOrDefault(state.start, Double.POSITIVE_INFINITY);
      double startG = state.g.getOrDefault(state.start, Double.POSITIVE_INFINITY);

      if (comparePair(v, startKey) >= 0 && Double.compare(startRhs, startG) == 0) {
        break;
      }

      state.open.remove(s);

      double gVal = state.g.getOrDefault(s, Double.POSITIVE_INFINITY);
      double rhsVal = state.rhs.getOrDefault(s, Double.POSITIVE_INFINITY);

      if (gVal > rhsVal) {
        state.g.put(s, rhsVal);
        state.closed.add(s);

        for (GridPosition sn : getOpenNeighbors(s, obstacles)) {
          updateState(sn, state, obstacles);
        }
      } else {
        state.g.put(s, Double.POSITIVE_INFINITY);
        for (GridPosition sn : getOpenNeighbors(s, obstacles)) {
          updateState(sn, state, obstacles);
        }
        updateState(s, state, obstacles);
      }
    }
  }

  private void updateState(GridPosition s, ADStarSegment state, Set<GridPosition> obstacles) {
    if (!s.equals(state.goal)) {
      state.rhs.put(s, Double.POSITIVE_INFINITY);

      double minRhs = Double.POSITIVE_INFINITY;
      for (GridPosition x : getOpenNeighbors(s, obstacles)) {
        double gVal = state.g.getOrDefault(x, Double.POSITIVE_INFINITY);
        double costVal = cost(s, x, obstacles);
        minRhs = Math.min(minRhs, gVal + costVal);
      }
      state.rhs.put(s, minRhs);
    }

    state.open.remove(s);

    double gVal = state.g.getOrDefault(s, Double.POSITIVE_INFINITY);
    double rhsVal = state.rhs.getOrDefault(s, Double.POSITIVE_INFINITY);

    if (Double.compare(gVal, rhsVal) != 0) {
      if (!state.closed.contains(s)) {
        state.open.put(s, key(s, state));
      } else {
        state.incons.put(s, Pair.of(0.0, 0.0));
      }
    }
  }

  private double cost(GridPosition sStart, GridPosition sEnd, Set<GridPosition> obstacles) {
    if (isCollision(sStart, sEnd, obstacles)) {
      return Double.POSITIVE_INFINITY;
    }
    return heuristic(sStart, sEnd);
  }

  private boolean isCollision(GridPosition sStart, GridPosition sEnd, Set<GridPosition> obstacles) {
    if (obstacles.contains(sStart) || obstacles.contains(sEnd)) {
      return true;
    }

    if (sStart.x != sEnd.x && sStart.y != sEnd.y) {
      GridPosition s1;
      GridPosition s2;

      if (sEnd.x - sStart.x == sStart.y - sEnd.y) {
        s1 = new GridPosition(Math.min(sStart.x, sEnd.x), Math.min(sStart.y, sEnd.y));
        s2 = new GridPosition(Math.max(sStart.x, sEnd.x), Math.max(sStart.y, sEnd.y));
      } else {
        s1 = new GridPosition(Math.min(sStart.x, sEnd.x), Math.max(sStart.y, sEnd.y));
        s2 = new GridPosition(Math.max(sStart.x, sEnd.x), Math.min(sStart.y, sEnd.y));
      }

      return obstacles.contains(s1) || obstacles.contains(s2);
    }

    return false;
  }

  private List<GridPosition> getOpenNeighbors(GridPosition s, Set<GridPosition> obstacles) {
    List<GridPosition> ret = new ArrayList<>();

    for (int xMove = -1; xMove <= 1; xMove++) {
      for (int yMove = -1; yMove <= 1; yMove++) {
        GridPosition sNext = new GridPosition(s.x + xMove, s.y + yMove);
        if (!obstacles.contains(sNext)
            && sNext.x >= 0
            && sNext.x < nodesX
            && sNext.y >= 0
            && sNext.y < nodesY) {
          ret.add(sNext);
        }
      }
    }
    return ret;
  }

  private List<GridPosition> getAllNeighbors(GridPosition s) {
    List<GridPosition> ret = new ArrayList<>();

    for (int xMove = -1; xMove <= 1; xMove++) {
      for (int yMove = -1; yMove <= 1; yMove++) {
        GridPosition sNext = new GridPosition(s.x + xMove, s.y + yMove);
        if (sNext.x >= 0 && sNext.x < nodesX && sNext.y >= 0 && sNext.y < nodesY) {
          ret.add(sNext);
        }
      }
    }
    return ret;
  }

  private Pair<Double, Double> key(GridPosition s, ADStarSegment state) {
    double gVal = state.g.getOrDefault(s, Double.POSITIVE_INFINITY);
    double rhsVal = state.rhs.getOrDefault(s, Double.POSITIVE_INFINITY);
    if (gVal > rhsVal) {
      return Pair.of(rhsVal + state.eps * heuristic(state.start, s), rhsVal);
    } else {
      return Pair.of(gVal + heuristic(state.start, s), gVal);
    }
  }

  private Pair<GridPosition, Pair<Double, Double>> topKey(ADStarSegment state) {
    Map.Entry<GridPosition, Pair<Double, Double>> min = null;
    for (var entry : state.open.entrySet()) {
      if (min == null || comparePair(entry.getValue(), min.getValue()) < 0) {
        min = entry;
      }
    }

    if (min == null) {
      return null;
    }

    return Pair.of(min.getKey(), min.getValue());
  }

  private double heuristic(GridPosition a, GridPosition b) {
    return Math.hypot(b.x - a.x, b.y - a.y);
  }

  private int comparePair(Pair<Double, Double> a, Pair<Double, Double> b) {
    int first = Double.compare(a.getFirst(), b.getFirst());
    if (first == 0) {
      return Double.compare(a.getSecond(), b.getSecond());
    } else {
      return first;
    }
  }

  private GridPosition getGridPos(Translation2d pos) {
    int x = (int) Math.floor(pos.getX() / nodeSize);
    int y = (int) Math.floor(pos.getY() / nodeSize);

    return new GridPosition(x, y);
  }

  private Translation2d gridPosToTranslation2d(GridPosition pos) {
    return new Translation2d(
        (pos.x * nodeSize) + (nodeSize / 2.0), (pos.y * nodeSize) + (nodeSize / 2.0));
  }

  //Decorates the pathplannerpath with actions based on zones and stop rotations
  private PathPlannerPath fillZones(PathPlannerPath basePath) {
    // Reset lists
    rotationTargets.clear();
    pointTowardsZones.clear();
    constraintZones.clear();
    eventMarkers.clear();

    // Path waypoints
    List<Waypoint> waypoints = basePath.getWaypoints();
    if (waypoints.size() < 2) return null;

    // Loop through all pathpoints. If it enters or exits a zone, get waypoint relative position
    List<PathPoint> points = basePath.getAllPathPoints();
    List<PathZone> activeZones = ZoneManager.getActiveZones();

    // //Since the rotation component of the stops was never injected into the path
    // //we have to artificialy create rotation targets at those points.

    double waypointNum = 0;
    for (Translation2d stop : requestRealStopPoses) {
      for (int i = 0; i < waypoints.size(); i++) {
          if (waypoints.get(i).anchor().equals(stop)) {
            waypointNum = i;
          }
      }
      rotationTargets.add(
        new RotationTarget(waypointNum, requestRealStopRotations.get(stop))
      );
    }

    //Actual zone filling in.

    for (PathZone zone : activeZones) {
      int entryIndex = -1;
      int exitIndex  = -1;

      for (int i = 0; i < points.size(); i++) {
        if (zone.containsPoint(points.get(i).position)) {
          if (entryIndex < 0) entryIndex = i;
          exitIndex = i;
        }
      }

      if (entryIndex < 0) continue;

      double entryWaypointIndex = points.get(entryIndex).waypointRelativePos;
      double exitWaypointIndex  = points.get(exitIndex).waypointRelativePos;

      if (zone instanceof OrientationZone oz) {
        pointTowardsZones.add(new PointTowardsZone(
            zone.name, 
            oz.getTarget().getTranslation(), 
            entryWaypointIndex, 
            exitWaypointIndex));
      } else if (zone instanceof RotationZone rz) {
        rotationTargets.add(new RotationTarget(
            entryWaypointIndex, 
            rz.getRotation()));

        rotationTargets.add(new RotationTarget(
            exitWaypointIndex, rz.getRotation()));
      } else if (zone instanceof ConstraintZone cz) {
        constraintZones.add(new ConstraintsZone(
            entryWaypointIndex,
            exitWaypointIndex,
            cz.getConstraints()));
      } else if (zone instanceof EventZone ez) {
        eventMarkers.add(new EventMarker(
            zone.name, 
            entryWaypointIndex, 
            exitWaypointIndex, 
            ez.getEvent()));
      }
    }

    basePath = new PathPlannerPath(
        waypoints,                                              
        new ArrayList<>(rotationTargets),                       
        new ArrayList<>(pointTowardsZones),                     
        new ArrayList<>(constraintZones),                       
        new ArrayList<>(eventMarkers),                          
        basePath.getGlobalConstraints(),                        
        null,                               
        basePath.getGoalEndState(),                              
        false                                          
    );

    return basePath;
  }

  /**
   * Represents a node in the pathfinding grid
   *
   * @param x X index in the grid
   * @param y Y index in the grid
   */
  public record GridPosition(int x, int y) implements Comparable<GridPosition> {
    @Override
    public int compareTo(GridPosition o) {
      if (x == o.x) {
        return Integer.compare(y, o.y);
      } else {
        return Integer.compare(x, o.x);
      }
    }
  }

  private record PathRequest(
      boolean needsReset,
      boolean doMinor,
      boolean doMajor,
      GridPosition sStart,
      List<GridPosition> sStops,
      GridPosition sGoal,
      Translation2d realStartPos,
      List<Translation2d> realStopPoses,
      Translation2d realGoalPos,
      Set<GridPosition> obstacles
  ) {}

  private static class ADStarSegment {
    GridPosition start;
    GridPosition goal;

    HashMap<GridPosition, Double> g = new HashMap<>();
    HashMap<GridPosition, Double> rhs = new HashMap<>();
    HashMap<GridPosition, Pair<Double, Double>> open = new HashMap<>();
    HashMap<GridPosition, Pair<Double, Double>> incons = new HashMap<>();
    Set<GridPosition> closed = new HashSet<>();

    double eps = EPS;

    ADStarSegment(GridPosition start, GridPosition goal) {
        this.start = start;
        this.goal = goal;
    }
  }
}

/*

#+- ..    . .*%@@@@@%%%##*.+****+:   .....::-..:::::.   .                                     ...:::-------::....  .......::---=+++**##%%%%%%*=+++===#
::..  .  .  ..    .     .. :-----.     ....:. ---:::: . ..                                             ...        .:::--===+++******##*++++*##**###**#
*=-::..:..  .. =.    ...   ......:      ..:...===-+++=-..                                  .:-:-:----:::...       ::--===++++++********###########%#%#
%%%%###+=-:...  +-:...:.  .====-+=:..:--==+:-*******=         .::-:-=====-=======-::.        :**#*******+=--::::..+*################*#########%%%@@@@#
%%%%%%%###*+#*####%##*=...-+++.:--..::-==++-+***+***-         .:..::::::::......            .=+***=**#*++==-:::...+*#%%%%###%%%%@@@@@@@@@@@@@@@@@@@@%#
@@@@@@@%%%%%%%%%%%%#*+..:.:+-...:..::-==++*+*#**+*****=-:                             ..:-=++++***-+****+=--:::::.=*#%%%#*:-*#%%%%@@@@@@@@@@@@@@@@@@@#
%@@@@@@@%%%%%%@@@%%#*=...:.:.......::--=++******+*+******+++++=-   =-:=-:--::----======++++++****+:--***++=-:::..::+*###**:.=*#%%%@%%@@@@@@@@@@@@@@@@#
@@@@@@%@@@@@@@@%%%#*+...::...... ..:--=+++++++++=+++++++======-:   .-= :#==--*+==-==---===========:..+===---::.....:+****=...**##%%%@@@@@@@@@@@@@@@@@#
@@%@%%%@%%%%%@@%##*+. ::::.:......::-=++****#*****++++==+=++==-     .+=. #-.=# ========--======++--::=++++=--::.... =+++-.....*#%%%%%%%%%%%%@@@@@@@@@#
@@@@@@@@%%%%%%#**+-. .::::::.::..:::-==+*+***+******+*++++***=    .  -*=-.= .= .-=--===---===-=--:-:--==+++=-::......+++- ....-+**####+=##%%%%%%@@@@@#
%%%%%%@%%##%%#*##*= :-:-::::..  ...::-===-::::.-+===--=:..   .   .  :=++*...    .: -.  *:-..... :::==-:.-=+=--::.....-++..:::..*#%%%%%%##%##%@@@@@@@%#
@@@%@%####%%%%###*..:::::.    : ...::::=:.::::. .:.::: .=.  :    .   =-#+#:.-   ...= .- .  .... .--=:.. .===--::..    :.  ...:-+*##%%%%%%%%%%%@%%%@%%#
%###%%%%%#######*= .:::.          ..:---:..:.:....:::...   .    .-    -:#@#..    .. - . =         . .:::..:-::.          ....  .*###%%%%%%%@@%%%%%%%%#
#%%%%########*##*+.                 ..    :::...   .+:.-.     .      ..-=+@@     ...-:  ::        . . .....:...           ...   =**#%%%%%@%%%%%%@@@%##
%%%%%%%%###**++=+=.                   ...  .      *+..- .-=.  - :.      +#=      ..:*:  .+       ........:...:....         ... . **###%%%%%%%%@@%%%%%#
##**===**###**#*=-                 ...::::::::++*#=-=*   + :  +.+ -   =.=+ .  .  . := .  *=-:-:-=-::...::-::.:::....             -++-:-*######%%%%####
*:       :+=---:..          .....::::-::--*+++*#-+===+.  + +  +-      =.-=     . .-:= . .-==-=+**+++=-----=--.:::::..   : .....   :...:::..  .+#+. ..#
=..:-:      :...:::        .:..:---==++++*++##*=---:-    :  = .  . *     :.    .-.*-=- .:*:=+++=+++*++*****++=+++=-:....  .....   :.-::::.     -: ...#
 ...::......:.  .::.  .- ....::...:-=+++#*##%=-=+:+**- .  . :=-  - :* -  .-  :  -#=#=+. =@* -++=======+=---=++++==---:--:::..     ::..::.     ......:#
.......  .......-==.   .   .::---=-*=+++###=*+***#* -+  :   .+    ..*+  - : .-  :=@@==.#%: -+-.::-:==-=+-------:=:...:::::::::.. .:::...      ..::.. #
:.  ...:-+-::=+=+++:   ..  . ....:=+:-=+*#=:-:=+++++ :+  . -      . +   . - .   -=#@* *   -*= -:-====-======-:.::::...::::::::.. -==++:      .:---.  #
.   :*#**+=+-:::----. .::  ....:: --===:#+=*---++-=-=  = -  . -   :+   .  + :   :-%=-   .-=--==.****+*++===*+-...:........:..    ---==++*+**+=:::... #
###+------. -=----=-- ::-. .....:--=+*:%+=+++-...=++ :    :  :  .  =   :  -.    *@@:. .:.**+-.+**+*=+**+++*****++=-.       .... .+++=:-=---===+=+++..#
+*+=-=++++==*:. .:..:. ..:  ..::::-++*##--+...: .:::+  :  :-   + :-- . .. =:   +%@.. +*+=*= +=++++++++=---===+++=---::.   ..     ==++***+**+=. ..:.::#
****-:=::... :+#%%####-**++ :---::-++**+=* : .=    . -      - -     =   = =:::@@@@@+*++.. ++#+**+++++==-=------=---::.. .:.... -.:--:===--::--*+***-.#
:.:-.-+*####%@%%%%%##**+-+= ..::::=+*++=*.....-+=::-: + .            :  +  :  @*-@@-#+-**+@:@--=----=-----::--::-::::..  -:    ===*#%%%*:-=:--+=++***#
###%%%#**+++===***++---- = .:::::-=+*+=-+:.-=*-   .-*-- =  -.-        :=.==    *=@@#++#%+*#*#%:::-=:-===+-=-=+--.. ...   ---   =-=**++--+#*++-. ..:-=#
*#*####%%*==******+++==  .   ..::-=***+-+-+:        ..%-+      .      ::::=%@   =#@+@@:%+%:***+#=:-:---=:::::-:--::.    .-:.  .:-=-:-=:-****#+=****:.#
%%%%%####*****+++.===:     ...:::-+#*#*-*+: .       .:=+*==.+-: .:   -:...:==%..#@#@:%.+*=---*.@@:-=+++====++====-..  . ----::::.:---==:::===--.*#***#
+*##%%%#**++++=.: .::           .-*######%         :.::=++*+-=% ::.:  -:..#.=@: .@-@*+*+*=+*%*@%+-=*#=*==--:++==-:-:::   :::==:== ::--::::-:::: :==++#
#####**#*++==:-            :--==+**##%%+%+        :=    :..%# %:    -::.. .=:  .@#%%.%*=---*----*-:=+%+-*=-+*+-=---:::.. :+=+:.....--=-:::::.    ::-:#
#**==+*-::.- :        .....:-:--:-+##%%##*        .      ..:=@ -  :..:.==++++=*=@+=%=-+--+   .:--+--+*++*:::-===-=--::... :.==++*. . .::::.       ---#
--*#****+==     .. ......:::---=:+#*##%#..:               .:.=%-*        *=:==-@===::: -   ...:-+*+***+++-+-:-....::.....  . -++- . . =-:     .   :::#
-#%%%*+++=:  . ......::.:::--==-:+#*###%* .       :       .:.==*-*   : = # -==%-@::.: :: :.:::-*#=-+%.-:++*=-:-::........    ::  .. ...--....     .:-#
*#*++*#*-.    .:.:::....:::----=+++#%#=+= =        =.:     .+.-:.*+   +==-+*+*--@:..:-   : --*.   :.:*%%%%%*..::--:::.                 ..  .:::..::::#
***+=---- .....  .....:::-------+%#+=-+==%=  -           :.: ..:==:= .    :=%*:@#-. .= ..::##      .::#%#=%+..    .::...        ...         .... . :-#
*==:--:..      ....:::::::-==+=+-+##*-+++*+=   -    :    :- =%%*-=.    = ++  -** :..+.:::-%:#      ..:++*%%-:::.....:.                              :#
+=+*+=-:    . .:::---::---------:-=: ...###*  :-   :+.. .:-:--#%%%-     =  ..:*...--- ..*@.:       :.:* +#%==----..                    :::: .      .:#
+**+-:::        ..::-::::...-----:..     *%#      ++.+ :-:::++@:#%*..  :=  ..*.::+#@-:.+      .   :..=  *#:----=-:.                   .--:. .      :-#
=-=====-           .. ...  .--==+=::  .  :@*:  :  .:  -    :=::::.=+=:.:= :.-+ :-@@%#=     :.:.  .:.+:=*%=:=--===-.......    .        :::    .    .--#
-.-==--=          ...:::---.   -:.: ==+:%@#:     =.        ... =    +:    * %@:-@%%@@     ...:= :===#*+@.%%%.::-=-:::.:...           .:- .:.     .=--#
- .-:--:                .::.        . -=#%%-+-  : :        .          = .:--+@#@%%%@     :..-=@   :::=:@@%= :.====..:::...           ....:::...  .:..#
:  =--.             .:---=:::.........   *+%*:+ :-#::.=:  @   :    .   ::.:-*=#@%%@-     :.==%  .   .=#%@      --=---::..               .::...:=.+++=#
.    :.::     .       :--.:.:.:.....  .   :@*#+=-- #:.   *   @      -:- ..--:-:%%@@- .-  ::-@-   -  .=*%:       ::---:...          .   ...... .-==***#
...  ::::   . :        .:..::::::......   -*%@-=.-  :    -  =:     ..- :=-- .   ..        -@    .=:-=*%*          ......           ......... ..===+**#
 ... .::: . ..:        ....::::::.........:##:  .    -   .  -:- ...::+      : :==   -      .-   ::-%@**:         .. . .                 ...... ===++*#
.... :.:.::.::: :       ..::...............+:-  :=--+..     ::.=.:+      .  . :::  *+ + ..::    +.==@.               ....                      -=====#
 .   ::.::..:::::.      ...:::.:::.....:      :=.= =.:-..=.:-:+:- .       :*#  ::+:++++#=      .::=+#                ...              . :::... .+*-:-#
... ::. ::::..:: .        .:.::......-         ::=:-. :*%=::::.+-:    ....:-   - :+####%      .+-=@+                 . ...      .      .::.... .++***#
. . -:   .::::::         ....:::....:             :.  # .#-:-#:   + ....-:     -.=.==+=+=:...=--@=@ :  . .....   . .. . ..::.... .        .   ..**##*#
.  ::-.   .:.:..         ..:::::.:.  .             ..:  + =:.:      .:+. .:  ..   .=.+##@*+#%@@@@=@@ :.....:.......... .............            ::   #
  ::--     :::           .....:::=      .           .:=*+ .  -.:::.. :.:-    .+:-  -.+-#@@@@@@@@ *=+@.- :.::.....:::::. ....   ...... .        ...   #
..-::.      .:           .:---:-:       .     .       ......  =.::-.::-   =::+* -:+:-::+@@@@% =-@#@*%--.:-==:.-:=-.............  ......       ..    .#
:::.:                 .:::-=---                  ........ =   -          +    :.*-.=+:+@@@@% +-+--#:*%-.:=-.:--=-....-::.::.:.....           :::    .#
.....                ::---=-= .             .   .... .... *=- - =..  -+::- =  .-=@@%-+*=-=. ....-=---=:.::::---:-:-=-===-::::::..... .       :-:::: .#
.   .     .         .::--=---+     .          =....-#@@@@*-::  .:+  :   :+#..+.:+ =          .-....==--::::.:::::::::.---==::::.::.:..       . --:   #
  .      ...         .:::::::::   = .  .    ..... -:=++=+=- =-::+:=- - .- : #=-: =         -#   -+..-#:*. :::::...:....:::::::--::::::.        -::   #
.        ..........      .:..: :   .=+: ......:::=---=#+@@@% =***:*=::-.++++             =:     ....-.%@=   .::....:-=-:--...::-:--::..         -    #
        ...:........:---:::.-- .                 ==-+##-*@@@#+@:   =+:-  @%.+          .+        .   *.%:  . ......:::----====------:...        .    #
       ...::.... .--==++=+====  *  ...        .--+%-:=+#@*--%+:......   .*.+ -   -    :               #=#:  .....:::==--:-----=-==--:..              #
     ..........-::-=--=--=++++=+   ::::==-    .-=#@#%= --# =::::..:....  :  - :       :                @@+.     ...::-:---=----:    .                #
        ....-:-::---:..=+====-====+     .....+=-:=.=-#=##@@-:::.. ...... .            .                =@=*...    .  ::::::: .::-==....              #
      -----:-:---=-+=+==-:=++=------:*===--=%@.-.=+@:*+:...:..:..         :      .                     :@==:  .       ...:::::---=--=-::..        .  #
     .::----===---+=-====***-==+=====---@@@*  =:**@#--. .:.:....      .     -      :                    #=-=::::.:.:--......:::::::...:.:.         . #
........::===--=+++**++++===---==----------#-+*-#=@=-.. :::::::..     .       #      .                  ---*..::::----:----:.       .:---::          #
......::--==::=*+++====+++=-:----------:::.-+::-=+:.:..  ... ..             =  -=+      =              -%#-=....:::::..:--==-----:. ..:::::..        #
.::-=+==+==--=::::::----:..::::::---::::..:..  :.:..::.. ...  .                    +                 .=@@-=+:....:::.:----=======--=-::====--::      #
:::---.:..:.===----:..:::-:-::.:::::::::...  .::::::...: .....                .-:   @:=:           ..*@@#-##@:.:::...:::::--------===+==-..::...     #
:.  .:-----==-=---:..:::--:-:::::::.....           .              .             ::+  = *..-        .@@-@ ==--=..:.:.::.::-==::-::-:.::::.......:.    #
    .::--==--:===- .::=-:------::.... .  . ..::....  .    .        .           .  =- .   -        =@@:= .%* -=- ... :.   .:::--:::::=+#+=:++---.. . .#
        :=--=---.....:-==------......  .  ..       ..   .                      .   := .    .     %=: + ..@@  +@=.. ...  ....::-:::-----======--......#
            .::. .  --::-::::::.....                                           . : .                +  .-=%  :+@:..:.    ...::::--===========++.  ...#
                   ...              ........... ....                           - .        ..       =  . . :  ..=*   .    :-:-=========+++=+=-=++. .  #
                  --:-::.      ..........:.::::::...  .                        .  =     .         =     -.=  .= =% ..     .--:::--==+++++==++-.... ..#
        .:.: :.  --.::....::--::.......... ....:......... .. .                     =     :      ..        :   -+@- ..  ....--=--==+---:=+=+++-....   #
       ...:.:   ==:.::::::::::::.......... .  ......  ...... ...                  .  =                 :  . .--@=::..::.::---===+====-=-=+++++*+=:   #
      .:. .-.  --: .::.::::::::::.....:..... ....:..........    ..             -    .  *:      :.        . -=*@:..:::.....::--====--------+++++=.    #
       ..:-   :::  :::.::=--:::-:................... .                          .-         .-.     -.--.-..+#@-.  ......:::::--:---::::::: :==+++.   #
       .--.. ::.  .::::=------:::::::......::. .:.....    ... .... ..              .=--:. .... ....  . ...:@@+.  ......::--------=======-:: .:-:=..  #
            .::   :.:-:-----=--:.:.:-..:::.  .:.....      .................           =-.....  ------=:. *@@:. ...::::::::::----=+--=----:- .:.:::.  #
             :...-++==-==:---=--- .:----...:..:::......   ........ ....      ..         ---..--+#@%+- -%@#=-.......:::--------:::-==--==--::-.  ...  #
                    -===--:---=---- :.:..::............ .  ..   .  ....                   -..--*-----#*@@@@*  ....:.:::-==--=-==---.         :   ..  #
                  .--===+=====-=--:    .          ..................               .            ##*%#-%*%@#-  ..::::----========.             .      #
                  .=++++++===:--:..:---:::::---:::...          ...    .              .....       :++*%**@@@:...:::::--: .----==       ..:.....       #
               ... :=+++=-:..:-======-=----:--:::::....-....   .........              . .......  =+%*++*#@%%.:.::::.::--    .-.    .. ..::... ..     #
        . .  .     ..:-::-==-----====-=---:----::::-.:.....   :..........    ........... ........ %+-:=%*#@@: .::::-:---:---.        .. .....   .    #
            .:.. ....-+==.+======----:-:-------------:::..   ::.......................:::.....::.:=+#*+**#*@*--  ::::-----::-:     . ......      .   #
         ............. -++++=+====- --::-::-=::: -:::::.:   .:...::..........::.:::::-:::::::::::--@#%%**-#=@*-::  :.::::---- .      ... ..     . .. #
        ...... ... ...=++++=====- --=-::. ....             .::..:.::.::::::...:::::::::::::::::::-:*##+-++:-+%+-=-  .-:.-====::  ...             ..  #
             ..       :*++==++ .-::: ....  .               .:.::..::::::::::--::::::------------:::=%++ :++=:#*=.-=   .=  .-==+-. ...             .. #
                     .:--=-  ==-.       ..                .---:: :::::..:.::---:::-:::-:--------==--@*=:  =-+-@:-::     .-  ..                       #
                           .+=+=-     .....         ..    :-:::        ...:--:::-:::--------------==+%+-. .. :=#+:-:                .                #
                           --::-: . .....:...        ....::::.         .....::-::::-::--:-.----------**:=  .=:.**.:::                         . .    #
        ...                . *+=::   .......      .....:-:....            ....::-:-------- --------:::=--   =:-.#:+---.                         .    #
  . ... ...                 .:....                   ..:: ..                ...:.::-------. =-=-- .....: :   +.:-*-++**....                          #
... ......                         .                       .                   .:---:-----: .-. .   . .  :.: ===.+. .......             . ....... .  #
:........                                                                       .-=====-. - . .            :::- = .                   .... :.... .=..#
**..=:..                             ..  .             .                         .:-++++==:                 . . .... . .     . ..          .%%%%%%%%%#
-+##:.                                 .                                          ..:-..                     . ...               . .. ..    .*%%%%%%%#
###=..                                                                            .. .    .                        . .               ..         %%%%%#
*#....             .                                                              .  ..                                                          #####
######################################################################################################################################################

*/
