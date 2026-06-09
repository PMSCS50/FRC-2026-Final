package frc.robot.commands;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

import frc.robot.pathfinding.PPLogger;

import com.ctre.phoenix6.swerve.SwerveRequest;

// *Follow up to pathfinding by PP_Align (runs after pathfinder finishes)
// ?Provides a more precise setpoint alignment using three ProfiledPIDControllers. 
// ?Average Error can go from 5cm and 5 degrees down to under 1cm and 0.5 degrees.
// ?Alignment with this looks perfectly flush in AdvantageScope, so it should be good enough irl

public class PostPathPreciseAlignment extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;

    private final double xy_kp       = 8.0;
    private final double xy_kd       = 0.05;

    private final double theta_kp    = 2.47;
    private final double theta_ki    = 0.04;
    private final double theta_kd    = 0.032;


    private final double maxLinVel = 1.5;
    private final double maxLinAcc = 3.0;
    private final double maxAngVel = 2 * Math.PI;
    private final double maxAngAcc = 4 * Math.PI;

    private final double xy_tolerance    = 0.01;
    private final double theta_tolerance = Math.toRadians(0.75);
    private final double linVelTolerance = 0.05;
    private final double angVelTolerance = 0.1;

    private final double settleTime = 0.1;
    private final double ffBounds   = 0.01;

    // private final double xy_kp       = 4.0;
    // private final double xy_kd       = 0.1;
    // private final double theta_kp    = 4.0;

    // private final double maxLinVel = 0.75;             // m/s
    // private final double maxLinAcc = 1.5;              // m/s²
    // private final double maxAngVel = Math.PI;          // rad/s
    // private final double maxAngAcc = 2*Math.PI;        // rad/s²

    // private final double xy_tolerance    = 0.005;
    // private final double theta_tolerance = Math.toRadians(.5);
    // private final double linVelTolerance = 0.05;
    // private final double angVelTolerance = 0.1;

    // private final double settleTime = 0.1;
    // private final double ffBounds = 0.05;

    private final double maxElapsedtime = 5;

    private final double dt = 0.02;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;

    private RobotConfig robotConfig;
    private ChassisSpeeds prevSpeeds;
    private double settleStartTime = -1;
    private double startTime;

    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
        new SwerveRequest.ApplyRobotSpeeds();

    public PostPathPreciseAlignment(
        CommandSwerveDrivetrain drivetrain,
        Pose2d targetPose
    ) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;

        TrapezoidProfile.Constraints translationConstraints =
            new TrapezoidProfile.Constraints(maxLinVel, maxLinAcc);

        TrapezoidProfile.Constraints rotationConstraints =
            new TrapezoidProfile.Constraints(maxAngVel, maxAngAcc);

        xController     = new ProfiledPIDController(xy_kp, 0, xy_kd, translationConstraints);
        yController     = new ProfiledPIDController(xy_kp, 0, xy_kd, translationConstraints);
        thetaController = new ProfiledPIDController(theta_kp, theta_ki, theta_kd, rotationConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError("PostPathPreciseAlignment: Failed to load RobotConfig" + e.getMessage(), false);
            robotConfig = null;
        }

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        var state = drivetrain.getState();

        xController.reset(state.Pose.getX(), state.Speeds.vxMetersPerSecond);
        yController.reset(state.Pose.getY(), state.Speeds.vyMetersPerSecond);
        thetaController.reset(state.Pose.getRotation().getRadians(), state.Speeds.omegaRadiansPerSecond);

        prevSpeeds       = new ChassisSpeeds();
        settleStartTime  = -1;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        var state = drivetrain.getState();
        Pose2d currentPose          = state.Pose;
        ChassisSpeeds currentSpeeds = state.Speeds;

        double vxField = xController.calculate(currentPose.getX(), targetPose.getX());
        double vyField = yController.calculate(currentPose.getY(), targetPose.getY());
        double omega   = thetaController.calculate(
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians()
        );

        ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vxField, vyField, omega, currentPose.getRotation()
        );

        double[] ffX = new double[4];
        double[] ffY = new double[4];

        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        if (robotConfig != null && distance > ffBounds) {
            double ax = (targetSpeeds.vxMetersPerSecond     - prevSpeeds.vxMetersPerSecond)     / dt;
            double ay = (targetSpeeds.vyMetersPerSecond     - prevSpeeds.vyMetersPerSecond)     / dt;
            double aa = (targetSpeeds.omegaRadiansPerSecond - prevSpeeds.omegaRadiansPerSecond) / dt;

            ChassisSpeeds chassisForces = new ChassisSpeeds(
                robotConfig.massKG * ax,   // X force (N)
                robotConfig.massKG * ay,   // Y force (N)
                robotConfig.MOI    * aa    // Torque (N*m)
            );

            Translation2d[] wheelForces = robotConfig.chassisForcesToWheelForceVectors(chassisForces);

            for (int i = 0; i < wheelForces.length; i++) {
                ffX[i] = wheelForces[i].getX();
                ffY[i] = wheelForces[i].getY();
            }
        }

        drivetrain.setControl(
            pathApplyRobotSpeeds
                .withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(ffX)
                .withWheelForceFeedforwardsY(ffY)
        );

        prevSpeeds = targetSpeeds;

        PPLogger.logTargetPose(targetPose);
        PPLogger.logVelocities(
            Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond),
            Math.hypot(targetSpeeds.vxMetersPerSecond,  targetSpeeds.vyMetersPerSecond),
            currentSpeeds.omegaRadiansPerSecond,
            targetSpeeds.omegaRadiansPerSecond
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            pathApplyRobotSpeeds
                .withSpeeds(new ChassisSpeeds())
                .withWheelForceFeedforwardsX(new double[4])
                .withWheelForceFeedforwardsY(new double[4])
        );
    }

    @Override
    public boolean isFinished() {
        double now = Timer.getFPGATimestamp();

        // *Safety timeout
        if (now - startTime > maxElapsedtime) {
            return true;
        }

        var state            = drivetrain.getState();
        Pose2d current       = state.Pose;
        ChassisSpeeds speeds = state.Speeds;

        double translationError = current.getTranslation().getDistance(targetPose.getTranslation());
        double rotationError    = Math.abs(
            current.getRotation().minus(targetPose.getRotation()).getRadians()
        );
        double linearVelocity   = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double angularVelocity  = Math.abs(speeds.omegaRadiansPerSecond);

        boolean withinTolerance =
            translationError < xy_tolerance  &&
            rotationError    < theta_tolerance &&
            linearVelocity   < linVelTolerance  &&
            angularVelocity  < angVelTolerance;

        if (withinTolerance) {
            if (settleStartTime < 0) settleStartTime = now;
            return (now - settleStartTime) >= settleTime;
        } else {
            settleStartTime = -1;
            return false;
        }
    }
}