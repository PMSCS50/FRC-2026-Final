package frc.robot.pathfinding;

// import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

import java.util.List;

/**
 * *PPLogger - Pathmaster Telemetry to AdvantageKit
 * ?Provides helper methods to be called in PathPlannerLogging
 */
public class PPLogger {

  // !Field widget (Elastic picks this up from SmartDashboard)
  private static final Field2d field = new Field2d();

  // !Raw NT4 publishers (split so Elastic can graph each series separately)
  private static final DoublePublisher actualVelPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/PathMaster/vel/actual")
          .publish();

  private static final DoublePublisher commandedVelPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/PathMaster/vel/commanded")
          .publish();

  private static final DoublePublisher actualAngVelPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/PathMaster/vel/actualAngular")
          .publish();

  private static final DoublePublisher commandedAngVelPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/PathMaster/vel/commandedAngular")
          .publish();

  private static final DoublePublisher inaccuracyPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/PathMaster/pathInaccuracy")
          .publish();


  // !Internal state for derived metrics
  private static Pose2d lastCurrentPose  = new Pose2d();
  private static Pose2d lastTargetPose   = new Pose2d();
  private static List<Pose2d> lastPathPoses = List.of();

  // !nitialization
  /**
   * *Call once in robotInit() (or in the static block of your drive subsystem)
   * *to register the Field2d with SmartDashboard so Elastic can find it.
   */
  public static void init() {
    SmartDashboard.putData("Field", field);
  }

  // !Public setters (mirrors PPLibTelemetry API)

  /**
   * *Publish actual and commanded velocities.
   *
   * @param actualVel       Actual chassis speed in m/s
   * @param commandedVel    Commanded chassis speed in m/s
   * @param actualAngVel    Actual angular velocity in rad/s
   * @param commandedAngVel Commanded angular velocity in rad/s
   */
  public static void logVelocities(
      double actualVel,
      double commandedVel,
      double actualAngVel,
      double commandedAngVel) {

    // *AdvantageKit — logged to .wpilog AND re-published to NT4 automatically
    Logger.recordOutput("Pathmaster/Velocity/actual",          actualVel);
    Logger.recordOutput("Pathmaster/Velocity/commanded",       commandedVel);
    Logger.recordOutput("Pathmaster/Velocity/actualAngular",   actualAngVel);
    Logger.recordOutput("Pathmaster/Velocity/commandedAngular",commandedAngVel);

    // *Raw NT4 publishers so Elastic line graphs see individual series
    actualVelPub.set(actualVel);
    commandedVelPub.set(commandedVel);
    actualAngVelPub.set(actualAngVel);
    commandedAngVelPub.set(commandedAngVel);
  }

  /**
   * // *Publish the robot's current pose.
   *
   * @param pose Current robot pose from odometry / vision estimator
   */
  public static void logCurrentPose(Pose2d pose) {
    lastCurrentPose = pose;

    Logger.recordOutput("Pathmaster/currentPose", pose);
    field.setRobotPose(pose);

    updatePathInnacuracy();
  }

  /**
   * // *Publish the active path being followed.
   * // ?Also updates the path progress metric.
   *
   * @param path The PathMasterPath currently being executed
   */
  public static void logActivePath(List<Pose2d> path) {
    Pose2d[] posesArray = path.toArray(new Pose2d[0]);

    Logger.recordOutput("Pathmaster/activePath", posesArray);
    field.getObject("activePath").setPoses(lastPathPoses);

    // updatePathProgress();
  }

  /**
   * // *Publish the immediate target pose on the path (the lookahead / setpoint).
   *
   * @param targetPose Target pose from the path follower controller
   */
  public static void logTargetPose(Pose2d targetPose) {
    lastTargetPose = targetPose;

    Logger.recordOutput("Pathmaster/targetPose", targetPose);
    field.getObject("targetPose").setPoses(targetPose);

    updatePathInnacuracy();
  }

  // !Derived metrics

  /**
   * // *Recomputes path inaccuracy (distance between current and target pose).
   * // ?Called automatically whenever current or target pose is updated.
   */
  private static void updatePathInnacuracy() {
    double inaccuracy = lastCurrentPose
        .getTranslation()
        .getDistance(lastTargetPose.getTranslation());

    Logger.recordOutput("Pathmaster/pathInaccuracy", inaccuracy);
    inaccuracyPub.set(inaccuracy);
  }

}