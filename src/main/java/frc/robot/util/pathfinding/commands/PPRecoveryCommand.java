package frc.robot.util.pathfinding.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.pathfinding.builders.GoingMerry;
import frc.robot.util.pathfinding.telemetry.PPLogging;

// * Generic Drive to a point command, copied from Team 4915 Spartronics
// * They used this as post path precise alignment when using Pathplanner's pathfinding capabilities
// * Can also work well as precise alignment after paths
public class PPRecoveryCommand extends Command{
    
    public Subsystem[] driveSubsystems;
    public final PathPlannerTrajectoryState endState;
    private PathFollowingController driveController = GoingMerry.getController();

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;

    private PPRecoveryCommand(PathPlannerTrajectoryState endState) {
        this.driveSubsystems = GoingMerry.getDriveRequirements();
        this.endState = endState;

        endTrigger = new Trigger(() -> {
            Pose2d diff = GoingMerry.getCurrentPose().relativeTo(endState.pose);

            boolean rotation = MathUtil.isNear(
                0.0, 
                diff.getRotation().getRotations(), 
                DriveConstants.kRotationTolerance.getRotations(), 
                0.0, 
                1.0
            );

            boolean position = diff.getTranslation().getNorm() < DriveConstants.kPositionTolerance.in(Meters);

            boolean speed = Math.hypot(GoingMerry.getCurrentSpeeds().vxMetersPerSecond, GoingMerry.getCurrentSpeeds().vyMetersPerSecond) < DriveConstants.kSpeedTolerance.in(MetersPerSecond);
            
            return rotation && position && speed;
        });

        endTriggerDebounced = endTrigger.debounce(DriveConstants.kEndTriggerDebounce.in(Seconds));
    }

    public static Command generateCommand(PathPlannerTrajectoryState endState){
        return new PPRecoveryCommand(endState);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Drive/DriveToPointReached", false);
        PPLogging.logTargetPose(endState.pose);
        PPLibTelemetry.setTargetPose(endState.pose);
    }

    @Override
    public void execute() {

        Logger.recordOutput("Drive/DriveToPointReached", endTrigger.getAsBoolean());

        ChassisSpeeds targetSpeeds = driveController.calculateRobotRelativeSpeeds(GoingMerry.getCurrentPose(), endState);
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
    public void end(boolean interrupted) {
        Logger.recordOutput("Drive/DriveToPointReached", endTrigger.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}