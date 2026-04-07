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
 */
public class Pathmaster {

    private final PathConstraints constraints;
    private final HashMap<String, Pose2d> waypoints = new HashMap<>();
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Pose2d> robotPose;
    private final RoronoaZoro zoro;

    public Pathmaster(CommandSwerveDrivetrain drivetrain,double vmax, double amax,double omegamax, double alphamax) {

        this.constraints = new PathConstraints(vmax, amax, omegamax, alphamax);
        this.drivetrain = drivetrain;
        this.robotPose = () -> drivetrain.getState().Pose;

        this.zoro = new RoronoaZoro();
    }

    /** Register a named field pose for use with gotoWaypoint(). */
    public void addWaypoint(String name, Pose2d pose) {
        waypoints.put(name, pose);
    }


    /**
     * Creates a rotation zone. When the robot paths through it,
     * it will rotate to and hold the given heading.
     */
    public void addRotationZone(String name, Translation2d min, Translation2d max, Rotation2d rotation, boolean active) {
        zoro.addZone(new RotationZone(name, min, max, rotation), active);
    }

    /**
     * Creates an orientation zone. When the robot paths through it,
     * it will continuously face the given target pose.
     */
    public void addOrientationZone(String name, Translation2d min, Translation2d max, Rotation2d rotation, boolean active){
        zoro.addZone(new OrientationZone(name, min, max, targetPose), active);
    }

    public void activateZone(String name) {
        zoro.setZoneState(name, true);
    }

    public void activateZones(String... names) {
        for (String name : names) zoro.setZoneState(name, true);
    }

    /** Activates only the named zones, and deactivates everything else. */
    public void activateOnly(String... names) {
        zoro.setAllZones(false);
        for (String name : names) zoro.setZoneState(name, true);
    }

    public void deactivateZone(String name) {
        zoro.setZoneState(name, false);
    }

    public void deactivateZones(String... names) {
        for (String name : names) zoro.setZoneState(name, false);
    }

    /** Deactivates only the named zones, and activates everything else. */
    public void deactivateOnly(String... names) {
        zoro.setAllZones(true);
        for (String name : names) zoro.setZoneState(name, false);
    }

    // -----------------------------------------------------------------------
    // Pathfinding Commands
    // -----------------------------------------------------------------------

    /** Pathfind to any field pose. */
    public Command makePathTo(Pose2d destination) {
        if (!AutoBuilder.isConfigured()) return Commands.none();
        return Commands.defer(
            () -> AutoBuilder.pathfindToPose(destination, constraints),
            Set.of(drivetrain)
        );
    }

    /** Pathfind to a registered waypoint. */
    public Command gotoWaypoint(String name) {
        if (!AutoBuilder.isConfigured() || !waypoints.containsKey(name))
            return Commands.none();
        return Commands.defer(
            () -> AutoBuilder.pathfindToPose(waypoints.get(name), constraints),
            Set.of(drivetrain)
        );
    }

    /**
     * Intended alignment pipeline.
     * pathFindToPose() is good but it is not very accurate at ending at the right place, usually with ~5cm error.
     * However, a predetermined .path file has much less error, around <1cm.
     * So this pathfinds to the start of the path, but then follows the path to the end for precise alingment.
     */
    public Command makePathToThen(String pathName) {
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
     * Team 4915 had this in their code, so I copied it.
     */
    public Command pathToNearestPose(List<Pose2d> candidates) {
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
    public Command pathToNearestWaypoint() {
        if (waypoints.isEmpty()) return Commands.none();
        return pathToNearestPose(waypoints.values().stream().toList());
    }

    /**
     * Pathfinding but only alignment. This should be a vision method. Dont use this.
     */
    public Command faceTargetPose(Pose2d destination, Pose2d faceTarget) {
        if (!AutoBuilder.isConfigured()) return Commands.none();
        return Commands.defer(() -> {
            Rotation2d facing = getRotationToPose(destination, faceTarget);
            Pose2d oriented = new Pose2d(destination.getTranslation(), facing);
            return AutoBuilder.pathfindToPose(oriented, constraints);
        }, Set.of(drivetrain));
    }

    /**
     * Cancels any currently running pathfinding command
     * should immediately stop the drivetrain.
     */
    public void cancelPathing() {
        Command current = drivetrain.getCurrentCommand();
        if (current != null) {
            current.cancel();
        }
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    /** Returns the rotation needed to face to a target. */
    public Rotation2d getRotationToPose(Pose2d from, Pose2d target) {
        Translation2d delta = target.getTranslation().minus(from.getTranslation());
        return new Rotation2d(delta.getX(), delta.getY());
    }

    /** Returns the nearest registered waypoint pose to the robot's current position. */
    public Pose2d getNearestWaypoint() {
        return waypoints.values().stream()
            .min(Comparator.comparingDouble(
                p -> p.getTranslation()
                       .getDistance(robotPose.get().getTranslation())
            ))
            .orElseThrow();
    }
}