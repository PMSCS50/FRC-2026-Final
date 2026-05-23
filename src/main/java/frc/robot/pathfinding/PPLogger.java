// package frc.robot.pathfinding;

// import com.pathplanner.lib.path.PathPlannerPath;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.networktables.DoublePublisher;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import org.littletonrobotics.junction.Logger;

// import java.util.List;

// /**
//  * PPLogger - PathPlanner telemetry bridge for AdvantageKit + Elastic Dashboard.
//  *
//  * <p>Replaces PPLibTelemetry with:
//  *   - AdvantageKit Logger.recordOutput() for all pose/velocity data (gives you log replay for free)
//  *   - Field2d published to SmartDashboard for Elastic's Field widget
//  *   - Separate NT4 topics for each velocity value so Elastic line graphs can plot them individually
//  *   - Path progress (0.0 → 1.0) computed from nearest waypoint index
//  *
//  * <p>Usage: Call the static setters from your drive subsystem / path following command,
//  * just like you would with PPLibTelemetry.
//  *
//  * <p>In Elastic, add:
//  *   - Field2d widget → key "Field"
//  *   - Line Graph → keys "/PathPlanner/vel/actual" + "/PathPlanner/vel/commanded"
//  *   - Line Graph → keys "/PathPlanner/vel/actualAngular" + "/PathPlanner/vel/commandedAngular"
//  *   - Line Graph → key "/PathPlanner/pathInaccuracy"
//  *   - Number Bar  → key "/PathPlanner/pathProgress" (min 0, max 1)
//  */
// public class PPLogger {

//   // ── Field widget (Elastic picks this up from SmartDashboard) ──────────────
//   private static final Field2d field = new Field2d();

//   // ── Raw NT4 publishers (split so Elastic can graph each series separately) ─
//   private static final DoublePublisher actualVelPub =
//       NetworkTableInstance.getDefault()
//           .getDoubleTopic("/PathPlanner/vel/actual")
//           .publish();

//   private static final DoublePublisher commandedVelPub =
//       NetworkTableInstance.getDefault()
//           .getDoubleTopic("/PathPlanner/vel/commanded")
//           .publish();

//   private static final DoublePublisher actualAngVelPub =
//       NetworkTableInstance.getDefault()
//           .getDoubleTopic("/PathPlanner/vel/actualAngular")
//           .publish();

//   private static final DoublePublisher commandedAngVelPub =
//       NetworkTableInstance.getDefault()
//           .getDoubleTopic("/PathPlanner/vel/commandedAngular")
//           .publish();

//   private static final DoublePublisher inaccuracyPub =
//       NetworkTableInstance.getDefault()
//           .getDoubleTopic("/PathPlanner/pathInaccuracy")
//           .publish();

//   private static final DoublePublisher pathProgressPub =
//       NetworkTableInstance.getDefault()
//           .getDoubleTopic("/PathPlanner/pathProgress")
//           .publish();

//   // ── Internal state for derived metrics ────────────────────────────────────
//   private static Pose2d lastCurrentPose  = new Pose2d();
//   private static Pose2d lastTargetPose   = new Pose2d();
//   private static List<Pose2d> lastPathPoses = List.of();

//   // ── Initialization ─────────────────────────────────────────────────────────

//   /**
//    * Call once in robotInit() (or in the static block of your drive subsystem)
//    * to register the Field2d with SmartDashboard so Elastic can find it.
//    */
//   public static void init() {
//     SmartDashboard.putData("Field", field);
//   }

//   // ── Public setters (mirrors PPLibTelemetry API) ───────────────────────────

//   /**
//    * Publish actual and commanded velocities.
//    *
//    * @param actualVel       Actual chassis speed in m/s
//    * @param commandedVel    Commanded chassis speed in m/s
//    * @param actualAngVel    Actual angular velocity in rad/s
//    * @param commandedAngVel Commanded angular velocity in rad/s
//    */
//   public static void setVelocities(
//       double actualVel,
//       double commandedVel,
//       double actualAngVel,
//       double commandedAngVel) {

//     // AdvantageKit — logged to .wpilog AND re-published to NT4 automatically
//     Logger.recordOutput("PathPlanner/vel/actual",          actualVel);
//     Logger.recordOutput("PathPlanner/vel/commanded",       commandedVel);
//     Logger.recordOutput("PathPlanner/vel/actualAngular",   actualAngVel);
//     Logger.recordOutput("PathPlanner/vel/commandedAngular",commandedAngVel);

//     // Raw NT4 publishers so Elastic line graphs see individual series
//     actualVelPub.set(actualVel);
//     commandedVelPub.set(commandedVel);
//     actualAngVelPub.set(actualAngVel);
//     commandedAngVelPub.set(commandedAngVel);
//   }

//   /**
//    * Publish the robot's current pose.
//    *
//    * @param pose Current robot pose from odometry / vision estimator
//    */
//   public static void setCurrentPose(Pose2d pose) {
//     lastCurrentPose = pose;

//     Logger.recordOutput("PathPlanner/currentPose", pose);
//     field.setRobotPose(pose);

//     updateDerivedMetrics();
//   }

//   /**
//    * Publish the active path being followed.
//    * Also updates the path progress metric.
//    *
//    * @param path The PathPlannerPath currently being executed
//    */
//   public static void setCurrentPath(PathPlannerPath path) {
//     lastPathPoses = path.getPathPoses();
//     Pose2d[] posesArray = lastPathPoses.toArray(new Pose2d[0]);

//     Logger.recordOutput("PathPlanner/activePath", posesArray);
//     field.getObject("activePath").setPoses(lastPathPoses);

//     updatePathProgress();
//   }

//   /**
//    * Publish the immediate target pose on the path (the lookahead / setpoint).
//    *
//    * @param targetPose Target pose from the path follower controller
//    */
//   public static void setTargetPose(Pose2d targetPose) {
//     lastTargetPose = targetPose;

//     Logger.recordOutput("PathPlanner/targetPose", targetPose);
//     field.getObject("targetPose").setPoses(targetPose);

//     updateDerivedMetrics();
//   }

//   // ── Derived metrics ───────────────────────────────────────────────────────

//   /**
//    * Recomputes path inaccuracy (distance between current and target pose).
//    * Called automatically whenever current or target pose is updated.
//    */
//   private static void updateDerivedMetrics() {
//     double inaccuracy = lastCurrentPose
//         .getTranslation()
//         .getDistance(lastTargetPose.getTranslation());

//     Logger.recordOutput("PathPlanner/pathInaccuracy", inaccuracy);
//     inaccuracyPub.set(inaccuracy);

//     updatePathProgress();
//   }

//   /**
//    * Computes path progress as a value from 0.0 (start) to 1.0 (end).
//    *
//    * <p>Strategy: find the index of the path waypoint closest to the robot's
//    * current position. That index divided by the total waypoint count gives a
//    * clean, monotonically-increasing progress value that works for any path shape.
//    *
//    * <p>Published to:
//    *   - AKit: "PathPlanner/pathProgress"
//    *   - NT4:  "/PathPlanner/pathProgress"  ← wire this to an Elastic Number Bar (0–1)
//    */
//   private static void updatePathProgress() {
//     if (lastPathPoses.isEmpty()) {
//       Logger.recordOutput("PathPlanner/pathProgress", 0.0);
//       pathProgressPub.set(0.0);
//       return;
//     }

//     int closestIndex = 0;
//     double closestDist = Double.MAX_VALUE;

//     for (int i = 0; i < lastPathPoses.size(); i++) {
//       double dist = lastCurrentPose
//           .getTranslation()
//           .getDistance(lastPathPoses.get(i).getTranslation());

//       if (dist < closestDist) {
//         closestDist = dist;
//         closestIndex = i;
//       }
//     }

//     // Normalize to [0, 1]. Use size-1 so the last waypoint == 1.0 exactly.
//     double progress = (double) closestIndex / Math.max(1, lastPathPoses.size() - 1);

//     Logger.recordOutput("PathPlanner/pathProgress", progress);
//     pathProgressPub.set(progress);
//   }
// }