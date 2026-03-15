import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

//EXPERIMENT FOR NEXT YEAR
//NOT FOR 2026 REBUILT
//Basically, this class can use PathPlanner's AutoBuilder to create a path on-the-fly between them and follow it.
//In simpler terms, we can comfortably go to ANY POSE ON THE FIELD while avoiding obstacles.

public class Pathfinder {

    private static final PathConstraints CONSTRAINTS;

    public Pathfinder(double vmax, double amax, double omegamax, double alphamax) {
        CONSTRAINTS = new PathConstraints(vmax, amax, omegamax, alphamax);
    }

    public Command makePathTo(Pose2d destination) {
        return AutoBuilder.pathfindToPose(destination, CONSTRAINTS, 0.0);
    }
}

//This is it. AutoBuilder does everything for us.

//Go to line 291 in RobotContainer.java for an application of this class.