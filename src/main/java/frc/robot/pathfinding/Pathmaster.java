package frc.robot.pathfinding;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Uses PathPlanner's pathfinding features to generate path commands to any position on the field.
 * Uses custom pathfinding class RoronoaZoro for zone-aware rotation.
 * must call initializePathfinder(), scheduleWarmup(), setDrivetrain(), and setConstraints() before using any other methods.
 */
public class Pathmaster {


    private static PathConstraints constraints;
    private static CommandSwerveDrivetrain drivetrain;
    private static Supplier<Pose2d> robotPose;
    private static RoronoaZoro zoro;
    private static final HashMap<String, Pose2d> waypoints = new HashMap<>();
    private static boolean configured = false;

    // --------
    // Configs
    // --------

    //Call in Robot.java before RobotContainer is initialized.
    public static void initializePathfinder() {
        Pathmaster.zoro = new RoronoaZoro();
        Pathfinding.setPathfinder(Pathmaster.zoro);
    }

    //Call in Robot.java as the last line in robotInit().
    public static void warmupCommand() {
        PathfindingCommand.warmupCommand().schedule();
    }

    //Call in RobotContainer.java
    public static void setDrivetrain(CommandSwerveDrivetrain swerve) {
        Pathmaster.drivetrain = swerve;
        Pathmaster.robotPose  = swerve::getPose;
    }

    //Call in RobotContainer.java
    public static void setConstraints(double vmax, double amax, double omegamax, double alphamax) {
        Pathmaster.constraints = new PathConstraints(vmax, amax, omegamax, alphamax);
    }


    /** Guards all methods — reports error and returns false if not fully configured. */
    private static boolean checkConfigured(String methodName) {
        if (!configured) {
            if (zoro == null || drivetrain == null ||
                    robotPose == null || constraints == null) {
                DriverStation.reportError(
                    "[Pathmaster] Not fully configured. " +
                    "Call initializePathfinder(), setDrivetrain(), " +
                    "setConstraints(), and scheduleWarmup() first. " +
                    "Method called: " + methodName, true);
                return false;
            }
            configured = true;
        }
        return true;
    }

    /** Register a named field pose for use with gotoWaypoint(). */
    public static void addWaypoint(String name, Pose2d pose) {
        waypoints.put(name, pose);
    }

    // ---------------
    // Zone Management
    // ---------------

    /**
     * Creates a rotation zone. When the robot paths through it,
     * it will rotate to and hold the given heading.
     */
    public static void addRotationZone(String name, Translation2d min, Translation2d max, Rotation2d rotation, boolean active) {
        if (!checkConfigured("addRotationZone")) return;
        zoro.addZone(new RotationZone(name, min, max, rotation), active);
    }

    /**
     * Creates an orientation zone. When the robot paths through it,
     * it will continuously face the given target pose.
     */
    public static void addOrientationZone(String name, Translation2d min, Translation2d max, Pose2d targetPose, boolean active) {
        if (!checkConfigured("addOrientationZone")) return;
        zoro.addZone(new OrientationZone(name, min, max, targetPose), active);
    }

    //Activates a single zone
    public static void activateZone(String name) {
        if (!checkConfigured("activateZone")) return;
        zoro.setZoneState(name, true);
    }

    //Activates multiple zones
    public static void activateZones(String... names) {
        if (!checkConfigured("activateZones")) return;
        for (String name : names) zoro.setZoneState(name, true);
    }

    /** Activates only the named zones, but deactivates everything else. */
    public static void activateOnly(String... names) {
        if (!checkConfigured("activateOnly")) return;
        zoro.setAllZones(false);
        for (String name : names) zoro.setZoneState(name, true);
    }

    //Deactivates a single zone
    public static void deactivateZone(String name) {
        if (!checkConfigured("deactivateZone")) return;
        zoro.setZoneState(name, false);
    }

    //Deactivates multiple zones
    public static void deactivateZones(String... names) {
        if (!checkConfigured("deactivateZones")) return;
        for (String name : names) zoro.setZoneState(name, false);
    }

    /** Deactivates only the named zones, but activates everything else. */
    public static void deactivateOnly(String... names) {
        if (!checkConfigured("deactivateOnly")) return;
        zoro.setAllZones(true);
        for (String name : names) zoro.setZoneState(name, false);
    }

    // --------------------
    // Pathfinding Commands
    // --------------------

    /** Pathfind to any field pose with obstacle avoidance. */
    public static Command makePathTo(Pose2d destination) {
        if (!checkConfigured("makePathTo")) return Commands.none();
        if (!AutoBuilder.isConfigured()) return Commands.none();
        return Commands.defer(
            () -> AutoBuilder.pathfindToPose(destination, constraints),
            Set.of(drivetrain)
        );
    }

    /** Pathfind to a registered waypoint. */
    public static Command gotoWaypoint(String name) {
        if (!checkConfigured("gotoWaypoint")) return Commands.none();
        if (!AutoBuilder.isConfigured() || !waypoints.containsKey(name))
            return Commands.none();
        return Commands.defer(
            () -> AutoBuilder.pathfindToPose(waypoints.get(name), constraints),
            Set.of(drivetrain)
        );
    }

    /**
     * Intended alignment pipeline.
     * pathfindToPose() has ~5cm error at endpoint.
     * A predetermined .path file has much less error, around <1cm.
     * This pathfinds to the start of the .path, then follows it precisely to the end.
     */
    public static Command makePathToThen(String pathName) {
        if (!checkConfigured("makePathToThen")) return Commands.none();
        if (!AutoBuilder.isConfigured()) return Commands.none();
        try {
            PathPlannerPath precisePath = PathPlannerPath.fromPathFile(pathName);
            return Commands.defer(
                () -> AutoBuilder.pathfindThenFollowPath(precisePath, constraints),
                Set.of(drivetrain)
            );
        } catch (Exception e) {
            DriverStation.reportError(
                "[Pathmaster] Path not found: " + pathName, true);
            return Commands.none();
        }
    }

    /**
     * Pathfinds to the nearest pose from a list of candidates.
     * Useful for "align to nearest scoring position" bindings.
     */
    public static Command pathToNearestPose(List<Pose2d> candidates) {
        if (!checkConfigured("pathToNearestPose")) return Commands.none();
        if (candidates.isEmpty()) return Commands.none();
        return Commands.defer(() -> {
            Pose2d nearest = candidates.stream()
                .min(Comparator.comparingDouble(
                    p -> p.getTranslation()
                           .getDistance(robotPose.get().getTranslation())
                ))
                .orElseThrow();
            return AutoBuilder.pathfindToPose(nearest, constraints);
        }, Set.of(drivetrain));
    }

    /** Pathfinds to the nearest registered waypoint. */
    public static Command pathToNearestWaypoint() {
        if (!checkConfigured("pathToNearestWaypoint")) return Commands.none();
        if (waypoints.isEmpty()) return Commands.none();
        return pathToNearestPose(waypoints.values().stream().toList());
    }

    /**
     * Pathfinds to a destination while arriving faced toward a separate target.
     * Note: heading is approximate — for exact heading use makePathToThen.
     */
    public static Command faceTargetPose(Pose2d destination, Pose2d faceTarget) {
        if (!checkConfigured("faceTargetPose")) return Commands.none();
        if (!AutoBuilder.isConfigured()) return Commands.none();
        return Commands.defer(() -> {
            Rotation2d facing = getRotationToPose(destination, faceTarget);
            Pose2d oriented = new Pose2d(destination.getTranslation(), facing);
            return AutoBuilder.pathfindToPose(oriented, constraints);
        }, Set.of(drivetrain));
    }

    /**
     * Cancels any currently running pathfinding command
     * and immediately stops the drivetrain.
     */
    public static Command cancelPathing() {
        if (!checkConfigured("cancelPathing")) return Commands.none();
        return Commands.runOnce(() -> {
            Command current = drivetrain.getCurrentCommand();
            if (current != null) {
                current.cancel();
            }
            drivetrain.drive(new ChassisSpeeds(0, 0, 0));
        });
    }

    // -------
    // Helpers
    // -------

    /** Returns the rotation needed at 'from' to face toward 'target'. */
    public static Rotation2d getRotationToPose(Pose2d from, Pose2d target) {
        Translation2d delta = target.getTranslation().minus(from.getTranslation());
        return new Rotation2d(delta.getX(), delta.getY());
    }

    /** Returns the nearest registered waypoint pose to the robot's current position. */
    public static Pose2d getNearestWaypoint() {
        if (!checkConfigured("getNearestWaypoint")) return new Pose2d();
        return waypoints.values().stream()
            .min(Comparator.comparingDouble(
                p -> p.getTranslation()
                       .getDistance(robotPose.get().getTranslation())
            ))
            .orElseThrow();
    }
}