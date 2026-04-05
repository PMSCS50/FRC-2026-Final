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

//Uses PathPlanner's pathfinding features to generate paths to any position on the field.
//We can use this to potentially revolutionize alignment in code.
//Can be very powerful if it works

public class Pathfinder {

    private final PathConstraints constraints;
    private final HashMap<String, Pose2d> waypoints = new HashMap<>();
    private final Supplier<Pose2d> robotPose;

    public Pathfinder(double vmax, double amax, double omegamax, double alphamax, Supplier<Pose2d> robotPose) {
        this.constraints = new PathConstraints(vmax, amax, omegamax, alphamax);
        this.robotPose = robotPose;
    }

    //Creates a waypoint on the field. Can mark important locations.    
    public void addWaypoint(String name, Pose2d pose) {
        waypoints.put(name, pose);
    }

    //Self explanatory. Creates a path to a destination pose and follows it.
    public Command makePathTo(Pose2d destination) {
        if (!AutoBuilder.isConfigured()) return Commands.none();
        //apparently we cant just return command, we must use commands.defer() or it breaks somehow. I dont fucking know how
        return Commands.defer(
            () -> AutoBuilder.pathfindToPose(destination, constraints),
            Set.of()
        );
    }

    //makePathTo() but it goes to a waypoint.
    public Command gotoWaypoint(String name) {
        if (!AutoBuilder.isConfigured() || !waypoints.containsKey(name)) return Commands.none();
        return Commands.defer(
            () -> AutoBuilder.pathfindToPose(waypoints.get(name), constraints),
            Set.of()
        );
    }

    //INTENDED ALIGNING PIPELINE
    //Pathplanning on-the-fly is not very accurate in the end state. However, .path files are more accurate.
    //Therefore, we can create a path that starts a little bit away but precisely ends at the target,
    //and then pathfind to that path. Theoretically most accurate idea. 
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

    //Team 4915 had this in their code so I copied it.
    //Given a bunch of poses, it travels to the nearest one.
    public Command pathToNearest(List<Pose2d> candidates) {
        if (candidates.isEmpty()) return Commands.none();
        return Commands.defer(() -> {
            Pose2d nearest = candidates.stream()
                .min(Comparator.comparingDouble(
                    p -> p.getTranslation()
                           .getDistance(robotPose.get().getTranslation())
                ))
                .orElseThrow();
            return AutoBuilder.pathfindToPose(nearest, constraints);
        }, Set.of());
    }

    //Same as above but to the nearest waypoint.
    public Command pathToNearestWaypoint() {
        if (waypoints.isEmpty()) return Commands.none();
        return pathToNearest(robotPose, waypoints.values().stream().toList());
    }

    //Uses PathPlanner for alignment. Please dont use this.
    //This current implementation isnt too precise.
    public Command faceTargetPose(Pose2d destination, Pose2d faceTarget) {
        if (!AutoBuilder.isConfigured()) return Commands.none();
        return Commands.defer(() -> {
            Rotation2d facing = getRotationToPose(destination, faceTarget);
            Pose2d oriented = new Pose2d(destination.getTranslation(), facing);
            return AutoBuilder.pathfindToPose(oriented, constraints);
        }, Set.of());
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    //Does exactly what you think it does.
    public Rotation2d getRotationToPose(Pose2d from, Pose2d target) {
        Translation2d delta = target.getTranslation().minus(from.getTranslation());
        return new Rotation2d(delta.getX(), delta.getY());
    }

}