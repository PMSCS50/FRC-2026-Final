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
 * Command factory that uses PathPlanner's pathfinding features to pathfind to a position on the field.
 * Uses custom pathfinding class RoronoaZoro for zone-aware rotation.
 * must call initializePathfinder() and scheduleWarmup() before using any other methods.
 */

public class Pathmaster {

    private PathConstraints constraints;
    private CommandSwerveDrivetrain drivetrain;
    private Supplier<Pose2d> robotPose;
    private Supplier<ChassisSpeeds> robotSpeeds;
    private static RoronoaZoroAK zoro;
    private final HashMap<String, Pose2d> waypoints = new HashMap<>();
    private static boolean configured = false;
    private static boolean warmup = false;

    //Used to prevent cancelPathing() from canceling other drivetrain commands.
    private List<String> commandNames = List.of(
        "makePathTo",
        "makePathToThen",
        "goToWaypoint",
        "pathToNearestPose",
        "pathToNearestWaypoint",
        "faceTargetPose"
    );

    // --------
    // Configs
    // --------

    public Pathmaster(
            CommandSwerveDrivetrain drivetrain,
            double vmax, 
            double amax, 
            double omegamax, 
            double alphamax ) {

        this.drivetrain = drivetrain;
        this.constraints = new PathConstraints(vmax, amax, omegamax, alphamax);
        this.robotPose = () -> drivetrain.getState().Pose;
        this.robotSpeeds = () -> drivetrain.getState().Speeds;

        linkStartingState();
    }

    //Call in Robot.java before RobotContainer is initialized.
    public static void initializePathfinder() {
        Pathmaster.zoro = new RoronoaZoroAK();
        Pathfinding.setPathfinder(Pathmaster.zoro);
    }

    //Call in Robot.java as the last line in robotInit().
    public static void startWarmupCommand() {
        PathfindingCommand.warmupCommand().schedule();
        warmup = true;
    }

    public IdealStartingState getIdealStartingState() {
        ChassisSpeeds speeds = robotSpeeds.get();
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        Rotation2d rot = robotPose.get().getRotation();
        return new IdealStartingState(Math.hypot(vx, vy), rot);
    }

    //Called in constructor to set RoronoaZoro IdealStartingStateSupplier
    private void linkStartingState() {
        zoro.setStartingStateSupplier(this::getIdealStartingState);
    }


    /** Guards all methods — reports error and returns false if not fully configured. */
    private boolean checkConfigured(String methodName) {
        if (!warmup || !AutoBuilder.isPathfindingConfigured()) {
            DriverStation.reportWarning(
                "[Pathmaster] Not configured when calling " + methodName +
                ". Call initializePathfinder() and startWarmupCommand() first.", false);
            return false;
        }
        return true;
    }

    /** Register a waypoint on the field. By calling gotoWaypoint() we can align here automatically. */
    public void addWaypoint(String name, Pose2d pose) {
        waypoints.put(name, pose);
    }

    // ---------------
    // Zone Management
    // ---------------

    /**
     * Creates a rotation zone. When the robot paths through it,
     * it will rotate to and hold the given heading.
     */
    public void addRotationZone(String name, Translation2d min, Translation2d max, Rotation2d rotation, boolean active) {
        if (!checkConfigured("addRotationZone")) return;
        ZoneManager.addZone(new RotationZone(name, min, max, rotation), active);
    }

    /**
     * Creates an orientation zone. When the robot paths through it,
     * it will continuously face the given target pose.
     */
    public void addOrientationZone(String name, Translation2d min, Translation2d max, Pose2d targetPose, boolean active) {
        if (!checkConfigured("addOrientationZone")) return;
        ZoneManager.addZone(new OrientationZone(name, min, max, targetPose), active);
    }

    //Activates a single zone
    public void activateZone(String name) {
        if (!checkConfigured("activateZone")) return;
        ZoneManager.setZoneState(name, true);
    }

    //Activates multiple zones
    public void activateZones(String... names) {
        if (!checkConfigured("activateZones")) return;
        for (String name : names) ZoneManager.setZoneState(name, true);
    }

    /** Activates only the named zones, but deactivates everything else. */
    public void activateOnly(String... names) {
        if (!checkConfigured("activateOnly")) return;
        ZoneManager.setAllZones(false);
        for (String name : names) ZoneManager.setZoneState(name, true);
    }

    //Deactivates a single zone
    public void deactivateZone(String name) {
        if (!checkConfigured("deactivateZone")) return;
        ZoneManager.setZoneState(name, false);
    }

    //Deactivates multiple zones
    public void deactivateZones(String... names) {
        if (!checkConfigured("deactivateZones")) return;
        for (String name : names) ZoneManager.setZoneState(name, false);
    }

    /** Deactivates only the named zones, but activates everything else. */
    public void deactivateOnly(String... names) {
        if (!checkConfigured("deactivateOnly")) return;
        ZoneManager.setAllZones(true);
        for (String name : names) ZoneManager.setZoneState(name, false);
    }

    // --------------------
    // Pathfinding Commands
    // --------------------

    /** Pathfind to any field pose with obstacle avoidance. */
    public Command makePathTo(Pose2d destination) {
        if (!checkConfigured("makePathTo")) return Commands.none();
        if (!AutoBuilder.isConfigured()) return Commands.none();
        return Commands.defer(
            () -> AutoBuilder.pathfindToPose(destination, constraints),
            Set.of(drivetrain)
        ).withName("makePathTo");
    }

    /** Pathfind to a registered waypoint. */
    public Command gotoWaypoint(String name) {
        if (!checkConfigured("gotoWaypoint")) return Commands.none();
        if (!AutoBuilder.isConfigured() || !waypoints.containsKey(name))
            return Commands.none();
        return Commands.defer(
            () -> AutoBuilder.pathfindToPose(waypoints.get(name), constraints),
            Set.of(drivetrain)
        ).withName("goToWaypoint");
    }

    /**
     * Intended alignment pipeline.
     * pathfindToPose() has ~5cm error at endpoint.
     * A predetermined .path file has much less error, around <1cm.
     * This pathfinds to the start of the .path, then follows it precisely to the end.
     */
    public Command makePathToThen(String pathName) {
        if (!checkConfigured("makePathToThen")) return Commands.none();
        if (!AutoBuilder.isConfigured()) return Commands.none();
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return Commands.defer(
                () -> AutoBuilder.pathfindThenFollowPath(path, constraints),
                Set.of(drivetrain)
            ).withName("makePathToThen");
        } catch (Exception e) {
            DriverStation.reportError(
                "[Pathmaster] Path not found: " + pathName, true);
            return Commands.none();
        }
    }

    /**
     * Pathfinds to the nearest pose from a list of candidates.
     * Copied from Spartronics.
     */
    public Command pathToNearestPose(List<Pose2d> candidates) {
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
        }, Set.of(drivetrain)).withName("pathToNearestPose");
    }

    /** Pathfinds to the nearest registered waypoint. */
    public Command pathToNearestWaypoint() {
        if (!checkConfigured("pathToNearestWaypoint")) return Commands.none();
        if (waypoints.isEmpty()) return Commands.none();
        return pathToNearestPose(waypoints.values().stream().toList()).withName("pathToNearestWaypoint");
    }

    /**
     * Pathfinds to a destination while arriving faced toward a separate target.
     * I dont think we should ever use this method over vision.
     */
    public Command faceTargetPose(Pose2d destination, Pose2d faceTarget) {
        if (!checkConfigured("faceTargetPose")) return Commands.none();
        if (!AutoBuilder.isConfigured()) return Commands.none();
        return Commands.defer(() -> {
            Rotation2d facing = getRotationToPose(destination, faceTarget);
            Pose2d oriented = new Pose2d(destination.getTranslation(), facing);
            return AutoBuilder.pathfindToPose(oriented, constraints);
        }, Set.of(drivetrain)).withName("faceTargetPose");
    }

    /**
     * Cancels any currently running pathfinding command
     * and immediately stops the drivetrain.
     */
    public Command cancelPathing() {
        if (!checkConfigured("cancelPathing")) return Commands.none();
        return Commands.runOnce(() -> {
            Command current = drivetrain.getCurrentCommand();
            if (current != null && commandNames.contains(current.getName())) {
                current.cancel();
            }
        });
    }

    // -------
    // Helpers
    // -------

    /** Returns the rotation needed at 'from' to face toward 'target'. */
    private Rotation2d getRotationToPose(Pose2d from, Pose2d target) {
        Translation2d delta = target.getTranslation().minus(from.getTranslation());
        return new Rotation2d(delta.getX(), delta.getY());
    }

    /** Returns the nearest registered waypoint pose to the robot's current position. */
    private Pose2d getNearestWaypoint() {
        if (!checkConfigured("getNearestWaypoint")) return new Pose2d();
        return waypoints.values().stream()
            .min(Comparator.comparingDouble(
                p -> p.getTranslation()
                       .getDistance(robotPose.get().getTranslation())
            ))
            .orElseThrow();
    }
}