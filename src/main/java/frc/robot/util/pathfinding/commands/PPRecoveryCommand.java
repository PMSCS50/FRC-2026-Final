package frc.robot.util.pathfinding.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.pathfinding.builders.GoingMerry;
import frc.robot.util.pathfinding.telemetry.PPLogging;
import frc.robot.util.pathfinding.util.PathUtil;

// * Modified Drive to Point Command that smoothly follows a path to a certain PPTrajState
// * Can be used to smoothly get back on the path if temporarily beached.
public class PPRecoveryCommand extends Command {
    
    private final PathPlannerTrajectoryState endState;
    private PathFollowingController pathController;

    private final Trigger endTrigger;

    private PPRecoveryCommand(PathPlannerPath ogPath, PathPlannerTrajectoryState endState) {

        this.endState = endState;

        this.pathController = new PPHolonomicDriveController(
            new PIDConstants(1.45, 0, 0.01),
            new PIDConstants(1, 0, 0));

        endTrigger = new Trigger(() -> {

            boolean atState = PathUtil.distanceAlongPath(ogPath, GoingMerry.getCurrentPose(), endState) <= 0.2;

            boolean rotation = MathUtil.isNear(
                endState.pose.getRotation().getDegrees(),
                GoingMerry.getCurrentPose().getRotation().getDegrees(), 
                10);

            Logger.recordOutput("Pathmaster/recovery ended/at state", atState);
            Logger.recordOutput("Pathmaster/recovery ended/rotation", rotation);
            return atState && rotation;

            }
        );
    }

    public static Command generateCommand(PathPlannerPath ogPath, PathPlannerTrajectoryState endState){
        return new PPRecoveryCommand(ogPath, endState);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Drive/DriveToPointReached", false);
        PPLogging.logTargetPose(endState.pose);
        PPLibTelemetry.setTargetPose(endState.pose);
        pathController.reset(GoingMerry.getCurrentPose(), GoingMerry.getCurrentSpeeds());

        PathPlannerAuto.currentPathName = "Recovery";
    }

    @Override
    public void execute() {

        Logger.recordOutput("Pathmaster/Recovered Path", endTrigger.getAsBoolean());

        ChassisSpeeds targetSpeeds = pathController.calculateRobotRelativeSpeeds(GoingMerry.getCurrentPose(), endState);
        GoingMerry.getOutput().accept(targetSpeeds, DriveFeedforwards.zeros(4));
        
        ChassisSpeeds currentSpeeds = GoingMerry.getCurrentSpeeds();

        PPLogging.logVelocities(
            Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond),
            Math.hypot(targetSpeeds.vxMetersPerSecond,  targetSpeeds.vyMetersPerSecond),
            currentSpeeds.omegaRadiansPerSecond,
            targetSpeeds.omegaRadiansPerSecond
        );

        
        PPLibTelemetry.setVelocities(
            Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond),
            Math.hypot(targetSpeeds.vxMetersPerSecond,  targetSpeeds.vyMetersPerSecond),
            currentSpeeds.omegaRadiansPerSecond,
            targetSpeeds.omegaRadiansPerSecond
        );
    }

    @Override
    public boolean isFinished() {
        return endTrigger.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Pathmaster/Recovered Path", endTrigger.getAsBoolean());
    }
}