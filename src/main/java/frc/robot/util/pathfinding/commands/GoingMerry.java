package frc.robot.util.pathfinding.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;
import java.util.function.*;

/** GoingMerry is what actually builds the pathfinding commands. Made to interact with ShinPathfindingCommand */
public class GoingMerry {
  private static Globals globals = new Globals();

  /**
   * Configures the GoingMerry for using PathPlanner's built-in commands.
   *
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   * @param robotRelativeSpeedsSupplier a supplier for the robot's current robot relative chassis
   *     speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
   *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
   *     using a differential drive, they will be in L, R order.
   *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize your
   *     module states, you will need to reverse the feedforwards for modules that have been flipped
   * @param controller Path following controller that will be used to follow paths
   * @param robotConfig The robot configuration
   * @param shouldFlipPath Supplier that determines if paths should be flipped to the other side of
   *     the field. This will maintain a global blue alliance origin.
   * @param driveRequirements the subsystem requirements for the robot's drive train
   */
  public static void configure(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
      BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Subsystem... driveRequirements) {
    if (globals.configured) {
      DriverStation.reportError(
          "Auto builder has already been configured. This is likely in error.", true);
    }

    globals.pathFollowingCommandBuilder =
        (path) ->
            new FollowPathCommand(
                path,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                output,
                controller,
                robotConfig,
                shouldFlipPath,
                driveRequirements);
    globals.poseSupplier = poseSupplier;
    globals.resetPose = resetPose;
    globals.configured = true;
    globals.shouldFlipPath = shouldFlipPath;
    globals.isHolonomic = robotConfig.isHolonomic;

    globals.pathfindToPoseCommandBuilder =
        (pose, constraints, goalEndVel) ->
            new ShinPathfindingCommand(
                pose,
                constraints,
                goalEndVel,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                output,
                controller,
                robotConfig,
                driveRequirements);

    globals.pathfindToPosesCommandBuilder =
        (pose, stops, constraints, goalEndVel) ->
            new ShinPathfindingCommand(
                pose,
                stops,
                constraints,
                goalEndVel,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                output,
                controller,
                robotConfig,
                driveRequirements);

    globals.pathfindThenFollowPathCommandBuilder =
        (path, constraints) ->
            new ShinPathfindingCommand(
                path,
                constraints,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                output,
                controller,
                robotConfig,
                shouldFlipPath,
                driveRequirements);

    globals.pathfindThroughStopsThenFollowPathCommandBuilder =
        (path, stops, constraints) ->
            new ShinPathfindingCommand(
                path,
                stops,
                constraints,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                output,
                controller,
                robotConfig,
                shouldFlipPath,
                driveRequirements);
    globals.pathfindingConfigured = true;
  }

  /**
   * Configures the GoingMerry for using PathPlanner's built-in commands.
   *
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   * @param robotRelativeSpeedsSupplier a supplier for the robot's current robot relative chassis
   *     speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds.
   * @param controller Path following controller that will be used to follow paths
   * @param robotConfig The robot configuration
   * @param shouldFlipPath Supplier that determines if paths should be flipped to the other side of
   *     the field. This will maintain a global blue alliance origin.
   * @param driveRequirements the subsystem requirements for the robot's drive train
   */
  public static void configure(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
      Consumer<ChassisSpeeds> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Subsystem... driveRequirements) {
    configure(
        poseSupplier,
        resetPose,
        robotRelativeSpeedsSupplier,
        (speeds, feedforwards) -> output.accept(speeds),
        controller,
        robotConfig,
        shouldFlipPath,
        driveRequirements);
  }

  /**
   * Holder for all global variables directly referenced by GoingMerry.
   *
   * <p>This class exists to ensure that {@link #resetForTesting()} resets all static state in
   * GoingMerry.
   */
  private static class Globals {
    boolean configured = false;

    Supplier<Pose2d> poseSupplier;
    Function<PathPlannerPath, Command> pathFollowingCommandBuilder;
    Consumer<Pose2d> resetPose;
    BooleanSupplier shouldFlipPath;
    boolean isHolonomic;

    // Pathfinding builders
    boolean pathfindingConfigured = false;
    TriFunction<Pose2d, PathConstraints, Double, Command> pathfindToPoseCommandBuilder;
    QuadFunction<Pose2d, List<Pose2d>, PathConstraints, Double, Command> pathfindToPosesCommandBuilder;

    BiFunction<PathPlannerPath, PathConstraints, Command> pathfindThenFollowPathCommandBuilder;
    TriFunction<PathPlannerPath, List<Pose2d>, PathConstraints, Command> pathfindThroughStopsThenFollowPathCommandBuilder;

  }

  /**
   * Returns whether the GoingMerry has been configured.
   *
   * @return true if the GoingMerry has been configured, false otherwise
   */
  public static boolean isConfigured() {
    return globals.configured;
  }

  /**
   * Returns whether the GoingMerry has been configured for pathfinding.
   *
   * @return true if the GoingMerry has been configured for pathfinding, false otherwise
   */
  public static boolean isPathfindingConfigured() {
    return globals.pathfindingConfigured;
  }

  /**
   * Resets {@code GoingMerry} static state to the values set at class initialization time.
   *
   * <p>This method should not be called during a competition.
   */
  public static void resetForTesting() {
    globals = new Globals();
  }

  /**
   * Get the current robot pose
   *
   * @return Current robot pose
   */
  public static Pose2d getCurrentPose() {
    return globals.poseSupplier.get();
  }

  /**
   * Get if a path or field position should currently be flipped
   *
   * @return True if path/positions should be flipped
   */
  public static boolean shouldFlip() {
    return globals.shouldFlipPath.getAsBoolean();
  }

  /**
   * Builds a command to follow a path. PathPlannerLib commands will also trigger event markers
   * along the way.
   *
   * @param path the path to follow
   * @return a path following command with for the given path
   * @throws AutoBuilderException if the GoingMerry has not been configured
   */
  public static Command followPath(PathPlannerPath path) {
    if (!isConfigured()) {
      throw new AutoBuilderException(
          "Auto builder was used to build a path following command before being configured");
    }

    return globals.pathFollowingCommandBuilder.apply(path);
  }

  /**
   * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
   * rotation and rotation delay distance will have no effect.
   *
   * @param pose The pose to pathfind to
   * @param constraints The constraints to use while pathfinding
   * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPose(
      Pose2d pose, PathConstraints constraints, double goalEndVelocity) {
    if (!isPathfindingConfigured()) {
      throw new AutoBuilderException(
          "Auto builder was used to build a pathfinding command before being configured");
    }

    return globals.pathfindToPoseCommandBuilder.apply(pose, constraints, goalEndVelocity);
  }

  public static Command pathfindToPose(
      Pose2d pose, List<Pose2d> stops, PathConstraints constraints, double goalEndVelocity) {
    if (!isPathfindingConfigured()) {
      throw new AutoBuilderException(
          "Auto builder was used to build a pathfinding command before being configured");
    }

    return globals.pathfindToPosesCommandBuilder.apply(pose, stops, constraints, goalEndVelocity);
  }

  /**
   * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
   * rotation and rotation delay distance will have no effect.
   *
   * @param pose The pose to pathfind to
   * @param constraints The constraints to use while pathfinding
   * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPose(
      Pose2d pose, PathConstraints constraints, LinearVelocity goalEndVelocity) {
    return pathfindToPose(pose, constraints, goalEndVelocity.in(MetersPerSecond));
  }

  /**
   * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
   * rotation and rotation delay distance will have no effect.
   *
   * @param pose The pose to pathfind to
   * @param constraints The constraints to use while pathfinding
   * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPose(
      Pose2d pose, List<Pose2d> stops, PathConstraints constraints, LinearVelocity goalEndVelocity) {
    return pathfindToPose(pose, stops, constraints, goalEndVelocity.in(MetersPerSecond));
  }

  /**
   * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
   * rotation will have no effect.
   *
   * @param pose The pose to pathfind to
   * @param constraints The constraints to use while pathfinding
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPose(Pose2d pose, PathConstraints constraints) {
    return pathfindToPose(pose, constraints, 0);
  }

  /**
   * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
   * rotation will have no effect.
   *
   * @param pose The pose to pathfind to
   * @param constraints The constraints to use while pathfinding
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPose(Pose2d pose, List<Pose2d> stops, PathConstraints constraints) {
    return pathfindToPose(pose, stops, constraints, 0);
  }


  /**
   * Build a command to pathfind to a given path, then follow that path. If not using a holonomic
   * drivetrain, the pose rotation delay distance will have no effect.
   *
   * @param goalPath The path to pathfind to, then follow
   * @param pathfindingConstraints The constraints to use while pathfinding
   * @return A command to pathfind to a given path, then follow the path
   */
  public static Command pathfindThenFollowPath(
      PathPlannerPath goalPath, PathConstraints pathfindingConstraints) {
    if (!isPathfindingConfigured()) {
      throw new AutoBuilderException(
          "Auto builder was used to build a pathfinding command before being configured");
    }

    return globals.pathfindThenFollowPathCommandBuilder.apply(goalPath, pathfindingConstraints);
  }

  /**
   * Build a command to pathfind to a given path, then follow that path. If not using a holonomic
   * drivetrain, the pose rotation delay distance will have no effect.
   *
   * @param goalPath The path to pathfind to, then follow
   * @param pathfindingConstraints The constraints to use while pathfinding
   * @return A command to pathfind to a given path, then follow the path
   */
  public static Command pathfindThenFollowPath(
      PathPlannerPath goalPath, List<Pose2d> stops, PathConstraints pathfindingConstraints) {
    if (!isPathfindingConfigured()) {
      throw new AutoBuilderException(
          "Auto builder was used to build a pathfinding command before being configured");
    }

    return globals.pathfindThroughStopsThenFollowPathCommandBuilder.apply(goalPath, stops, pathfindingConstraints);
  }
  
  /**
   * Get if GoingMerry was configured for a holonomic drive train
   *
   * @return True if holonomic
   */
  public static boolean isHolonomic() {
    if (!GoingMerry.isConfigured()) {
      throw new RuntimeException("GoingMerry was not configured before use");
    }

    return globals.isHolonomic;
  }

  

  /** Functional interface for a function that takes 3 inputs */
  @FunctionalInterface
  public interface TriFunction<In1, In2, In3, Out> {
    /**
     * Apply the inputs to this function
     *
     * @param in1 Input 1
     * @param in2 Input 2
     * @param in3 Input 3
     * @return Output
     */
    Out apply(In1 in1, In2 in2, In3 in3);
  }

  /** Functional interface for a function that takes 3 inputs */
  @FunctionalInterface
  public interface QuadFunction<In1, In2, In3, In4, Out> {
    /**
     * Apply the inputs to this function
     *
     * @param in1 Input 1
     * @param in2 Input 2
     * @param in3 Input 3
     * @param in4 Input 4
     * @return Output
     */
    Out apply(In1 in1, In2 in2, In3 in3, In4 in4);
  }
}