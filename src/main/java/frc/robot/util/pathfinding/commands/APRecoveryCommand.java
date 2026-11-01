
package frc.robot.util.pathfinding.commands;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.util.pathfinding.builders.GoingMerry;
import frc.robot.util.pathfinding.telemetry.PPLogging;

// Modified Drive to Point Command that uses Autopilot to recover to a PathPlanner trajectory state.
public class APRecoveryCommand extends Command {

    private final PathPlannerTrajectoryState endState;
    @SuppressWarnings("unused")
    private final PathPlannerPath ogPath;

    private final Autopilot autopilot;
    private final APTarget target;

    private final PIDController headingController;

    private final Trigger endTrigger;

    private APRecoveryCommand(
        PathPlannerPath ogPath,
        PathPlannerTrajectoryState endState,
        Autopilot autopilot
    ) {

        this.ogPath = ogPath;
        this.endState = endState;
        this.autopilot = autopilot;

        this.target = new APTarget(endState.pose)
            .withVelocity(endState.linearVelocity)
            .withEntryAngle(endState.heading);

        headingController = new PIDController(4.0, 0, 0);
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        endTrigger = new Trigger(() -> {

            boolean atState = autopilot.atTarget(GoingMerry.getCurrentPose(), target);

            Logger.recordOutput("Pathmaster/recovery ended/at state", atState);

            return atState;
        });
    }

    public static Command generateCommand(
        PathPlannerPath path,
        PathPlannerTrajectoryState endState
    ) {
        return new APRecoveryCommand(
            path, 
            endState, 
            new Autopilot(
                new APProfile(
                    new APConstraints(path.getGlobalConstraints().maxAccelerationMPSSq(), 2.0))
                    .withErrorXY(Centimeter.of(20))
                    .withErrorTheta(Degrees.of(10))
                    .withBeelineRadius(Meters.of(0.75))
            )
        );
    }

    @Override
    public void initialize() {

        Logger.recordOutput("Drive/DriveToPointReached", false);

        Logger.recordOutput(
            "Drive/DriveToPointMap",
            new Pose2d[] {
                GoingMerry.getCurrentPose(),
                endState.pose
            }
        );
        Logger.recordOutput(
            "Drive/DriveToPointReached",
            false
        );
        
        headingController.reset();


        PPLogging.logTargetPose(endState.pose);
        PPLibTelemetry.setTargetPose(endState.pose);
        PathPlannerAuto.currentPathName = "Recovery";
    }

    @Override
    public void execute() {

        Pose2d currentPose = GoingMerry.getCurrentPose();
        ChassisSpeeds currentSpeeds = GoingMerry.getCurrentSpeeds().times(10.0 / 3.0);

        // AutoPilot returns field-relative translational velocities
        // and a desired robot heading.
        APResult result = autopilot.calculate(
            currentPose,
            currentSpeeds,
            target
        );

        // AutoPilot does not calculate angular velocity.
        // Use a continuous PID controller to track its heading setpoint.
        double omega = headingController.calculate(
            currentPose.getRotation().getRadians(),
            result.targetAngle().getRadians()
        );

        // Convert AutoPilot's field-relative velocities into
        // the robot-relative speeds expected by GoingMerry.
        ChassisSpeeds targetSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                result.vx(),
                result.vy(),
                RadiansPerSecond.of(omega),
                currentPose.getRotation()
            );

        GoingMerry.getOutput().accept(
            targetSpeeds,
            DriveFeedforwards.zeros(4)
        );

        Logger.recordOutput(
            "Pathmaster/Recovered Path",
            endTrigger.getAsBoolean()
        );

        PPLogging.logVelocities(
            Math.hypot(
                currentSpeeds.vxMetersPerSecond,
                currentSpeeds.vyMetersPerSecond
            ),
            Math.hypot(
                targetSpeeds.vxMetersPerSecond,
                targetSpeeds.vyMetersPerSecond
            ),
            currentSpeeds.omegaRadiansPerSecond,
            targetSpeeds.omegaRadiansPerSecond
        );

        PPLibTelemetry.setVelocities(
            Math.hypot(
                currentSpeeds.vxMetersPerSecond,
                currentSpeeds.vyMetersPerSecond
            ),
            Math.hypot(
                targetSpeeds.vxMetersPerSecond,
                targetSpeeds.vyMetersPerSecond
            ),
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
        Logger.recordOutput(
            "Drive/DriveToPointReached",
            endTrigger.getAsBoolean()
        );
        Logger.recordOutput(
            "Pathmaster/Recovered Path",
            endTrigger.getAsBoolean()
        );
    }
}
