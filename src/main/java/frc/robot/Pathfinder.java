package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;


public class Pathfinder {

    private final PathConstraints constraints;
    private final HashMap<String, Pose2d> waypoints = new HashMap<>();
    private final Supplier<Pose2d> robotPose;

    public Pathfinder(double vmax, double amax, double omegamax, double alphamax, Supplier<Pose2d> robotPose) {
        this.constraints = new PathConstraints(vmax, amax, omegamax, alphamax);
        this.robotPose = robotPose;
    }

    public void addWaypoint(String name, Pose2d pose) {
        waypoints.put(name, pose);
    }

    public Command makePathTo(Pose2d destination) {
        if (!AutoBuilder.isConfigured()) return Commands.none();
        return Commands.defer(
            () -> AutoBuilder.pathfindToPose(flip(destination), constraints),
            Set.of()
        );
    }

    public Command gotoWaypoint(String name) {
        if (!AutoBuilder.isConfigured() || !waypoints.containsKey(name)) return Commands.none();
        return Commands.defer(
            () -> AutoBuilder.pathfindToPose(flip(waypoints.get(name)), constraints),
            Set.of()
        );
    }

    public Command makePathToThen(String pathName) {
        if (!AutoBuilder.isConfigured()) return Commands.none();
        try {
            PathPlannerPath precisePath = PathPlannerPath.fromPathFile(pathName);
            return Commands.defer(
                () -> AutoBuilder.pathfindThenFollowPath(precisePath, constraints),
                Set.of()
            );
        } catch (Exception e) {
            DriverStation.reportError("[Pathfinder] Path not found: " + pathName, true);
            return Commands.none();
        }
    }

    public Command pathToNearest(List<Pose2d> candidates) {
        if (candidates.isEmpty()) return Commands.none();
        return Commands.defer(() -> {
            Pose2d nearest = candidates.stream()
                .min(Comparator.comparingDouble(
                    p -> p.getTranslation()
                           .getDistance(robotPose.get().getTranslation())
                ))
                .orElseThrow();
            return AutoBuilder.pathfindToPose(flip(nearest), constraints);
        }, Set.of());
    }

    public Command pathToNearestWaypoint() {
        if (waypoints.isEmpty()) return Commands.none();
        return pathToNearest(robotPose, waypoints.values().stream().toList());
    }

    public Command faceTargetPose(Pose2d destination, Pose2d faceTarget) {
        if (!AutoBuilder.isConfigured()) return Commands.none();
        return Commands.defer(() -> {
            Rotation2d facing = getRotationToPose(destination, faceTarget);
            Pose2d oriented = new Pose2d(destination.getTranslation(), facing);
            return AutoBuilder.pathfindToPose(flip(oriented), constraints);
        }, Set.of());
    }

    // public Command pathToWithHeading(
    //         Supplier<ChassisSpeeds> robotSpeeds,
    //         Pose2d destination) {

    //     return Commands.defer(() -> {
    //         Pose2d robot = robotPose.get();
    //         ChassisSpeeds speeds = robotSpeeds.get();

    //         Translation2d delta = destination.getTranslation()
    //                                          .minus(robot.getTranslation());
    //         Rotation2d travelDir = new Rotation2d(delta.getX(), delta.getY());

    //         double currentSpeed = Math.hypot(
    //             speeds.vxMetersPerSecond,
    //             speeds.vyMetersPerSecond
    //         );

    //         List<Waypoint> wpList = PathPlannerPath.waypointsFromPoses(
    //             new Pose2d(robot.getTranslation(), travelDir),
    //             new Pose2d(destination.getTranslation(), travelDir)
    //         );

    //         PathPlannerPath path = new PathPlannerPath(
    //             wpList,
    //             constraints,
    //             new IdealStartingState(
    //                 currentSpeed,           // robot's actual current speed
    //                 robot.getRotation()     // robot's actual current heading
    //             ),
    //             new GoalEndState(0.0, destination.getRotation())
    //         );
    //         path.preventFlipping = true;

    //         return AutoBuilder.followPath(path);
    //     }, Set.of());
    // }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    public Rotation2d getRotationToPose(Pose2d from, Pose2d target) {
        Translation2d delta = target.getTranslation().minus(from.getTranslation());
        return new Rotation2d(delta.getX(), delta.getY());
    }

    private Pose2d flip(Pose2d pose) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            return FlippingUtil.flipFieldPose(pose);
        }
        return pose;
    }
}