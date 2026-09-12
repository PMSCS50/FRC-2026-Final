package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.util.pathfinding.telemetry.PPLogging;

// * Generic Drive to a point command, copied from Team 4915 Spartronics (their use of Pathplanner in 2025 convinced me to make PP Align)
// * Can also work well as precise alignment after paths
public class DriveToPointCommand extends Command{
    
    public CommandSwerveDrivetrain drivetrain;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = DriveConstants.driveController;

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;

    private final BooleanPublisher endTriggerLogger = NetworkTableInstance.getDefault().getTable("logging").getBooleanTopic("PositionPIDEndTrigger").publish();

    private DriveToPointCommand(CommandSwerveDrivetrain drivetrain, Pose2d goalPose) {
        this.drivetrain = drivetrain;
        this.goalPose = goalPose;

        endTrigger = new Trigger(() -> {
            Pose2d diff = drivetrain.getPose().relativeTo(goalPose);

            boolean rotation = MathUtil.isNear(
                0.0, 
                diff.getRotation().getRotations(), 
                DriveConstants.kRotationTolerance.getRotations(), 
                0.0, 
                1.0
            );

            boolean position = diff.getTranslation().getNorm() < DriveConstants.kPositionTolerance.in(Meters);

            boolean speed = Math.hypot(drivetrain.getSpeeds().vxMetersPerSecond, drivetrain.getSpeeds().vyMetersPerSecond) < DriveConstants.kSpeedTolerance.in(MetersPerSecond);

            System.out.println("end trigger conditions R: "+ rotation + "\tP: " + position + "\tS: " + speed);
            
            return rotation && position && speed;
        });

        endTriggerDebounced = endTrigger.debounce(DriveConstants.kEndTriggerDebounce.in(Seconds));
    }

    public static Command generateCommand(CommandSwerveDrivetrain swerve, Pose2d goalPose, Time timeout){
        return new DriveToPointCommand(swerve, goalPose).withTimeout(timeout).finallyDo(() -> {
            swerve.runVelocity(new ChassisSpeeds());
            swerve.applyRequest(() -> DriveConstants.xBrake);
        });
    }

    @Override
    public void initialize() {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
        PPLogging.logTargetPose(goalPose);
        PPLibTelemetry.setTargetPose(goalPose);
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        endTriggerLogger.accept(endTrigger.getAsBoolean());

        ChassisSpeeds targetSpeeds = mDriveController.calculateRobotRelativeSpeeds(drivetrain.getPose(), goalState);
        drivetrain.runVelocity(targetSpeeds);
        
        ChassisSpeeds currentSpeeds = drivetrain.getSpeeds();

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
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}