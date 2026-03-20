package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import static java.lang.Math.abs;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;

//EXPERIMENT FOR NEXT YEAR
//NOT NECESSARILY FOR 2026 REBUILT
//Basically, this class can use PathPlanner's AutoBuilder to create a path on-the-fly between two poses and follow it.
//In simpler terms, we can comfortably go to ANY POSE ON THE FIELD while avoiding obstacles.

public class Pathfinder {

    public static PathConstraints CONSTRAINTS;
    private CommandSwerveDrivetrain drivetrain;
    public Pose2d ClimbPose = Constants.VisionConstants.getClimbPose();

    public Pathfinder(double vmax, double amax, double omegamax, double alphamax, CommandSwerveDrivetrain drivetrain) {
        CONSTRAINTS = new PathConstraints(vmax, amax, omegamax, alphamax);
        this.drivetrain = drivetrain;
    }

    public Optional<PathPlannerPath> getClosestPath(CommandSwerveDrivetrain drivetrain) {
        List<String> pathNames = List.of("Far Left Trench", "Far Right Trench", "Close Left Trench", "Close Right Trench");

        PathPlannerPath closestPath = null;
        double minDist = Double.MAX_VALUE;
        //String closestPathName = "";

        boolean shouldFlip = DriverStation.getAlliance()
            .map(alliance -> alliance == DriverStation.Alliance.Red)
            .orElse(false);
        SmartDashboard.putBoolean("Path Flipped", shouldFlip);

        for (String name : pathNames) {
            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile(name);
                PathPlannerPath pathForDistance = shouldFlip ? path.flipPath() : path;

                List<Pose2d> poses = pathForDistance.getPathPoses();
                double dist = drivetrain.getPose().getTranslation()
                    .getDistance(poses.get(0).getTranslation());
                if (dist < minDist && abs(drivetrain.getPose().getX() - ClimbPose.getX()) > abs(poses.get(0).getX() - ClimbPose.getX())) {
                    minDist = dist;
                    closestPath = path;
                    //closestPathName = name;
                    //System.out.println("Closest path: " + name + " with distance: " + dist);
                } 
            } catch (IOException | ParseException e) {
                DriverStation.reportError("Failed to load path: " + name, false);
                return Optional.empty();
            }
        }
        //SmartDashboard.putString("ClosestPath", "-------------Closest path: " + closestPathName + " with distance: " + minDist);
        return Optional.ofNullable(closestPath);
    }

    public Command makePathTo(Pose2d destination) {
        SmartDashboard.putBoolean("AutoBuilderConfigured", AutoBuilder.isConfigured());
        return AutoBuilder.pathfindToPose(destination, CONSTRAINTS);
    }

    public Command pathToClimb(/*Optional<PathPlannerPath> closestPathOpt*/) {
        //Optional<PathPlannerPath> closestPathOpt = getClosestPath(drivetrain);
        Optional<PathPlannerPath> closestPathOpt = getClosestPath(drivetrain);
    
        if (closestPathOpt.isPresent()) {
            return AutoBuilder.pathfindThenFollowPath(closestPathOpt.get(), CONSTRAINTS);
        } else {
            SmartDashboard.putString("PathfinderError", "No valid path found!");
            try {
                return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Climb"), CONSTRAINTS);
            } catch (IOException | ParseException e) {
                DriverStation.reportError("Failed to load Climb path", false);
                return null;
            }
        }
    }
}

//This is it. AutoBuilder does everything for us.

//Go to line 327 in RobotContainer.java for an application of this class.