package frc.robot.pathfinding;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class PreciseAlignmentCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;

    // --- Tuning ---
    private static final double TRANSLATION_kP = 4.0;
    private static final double TRANSLATION_kD = 0.1;
    private static final double ROTATION_kP    = 4.0;

    // Correction-scale constraints — much tighter than PathPlanner travel values
    private static final double MAX_CORRECTION_SPEED     = 0.75;         // m/s
    private static final double MAX_CORRECTION_ACCEL     = 1.5;          // m/s²
    private static final double MAX_CORRECTION_ANG_SPEED = Math.PI;      // rad/s
    private static final double MAX_CORRECTION_ANG_ACCEL = Math.PI * 2;  // rad/s²

    // Finish tolerances
    private static final double POSITION_TOLERANCE_METERS  = 0.005;
    private static final double ROTATION_TOLERANCE_RADIANS = Math.toRadians(1.0);
    private static final double LINEAR_VELOCITY_TOLERANCE  = 0.05;
    private static final double ANGULAR_VELOCITY_TOLERANCE = 0.1;

    // Robot must hold tolerance for this long before finishing
    private static final double SETTLE_TIME_SECONDS = 0.1;

    // Command aborts after this regardless of tolerance
    private static final double MAX_DURATION_SECONDS = 2.5;

    // Only apply feedforward outside this radius — FF is noise at close range
    private static final double FF_DEADZONE_METERS = 0.05;

    // Loop period
    private static final double DT = 0.02;

    // --- Controllers ---
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;

    // --- State ---
    private RobotConfig robotConfig;
    private ChassisSpeeds prevSpeeds;
    private double settleStartTime = -1;
    private double commandStartTime;

    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
        new SwerveRequest.ApplyRobotSpeeds();

    public PreciseAlignmentCommand(
        CommandSwerveDrivetrain drivetrain,
        Pose2d targetPose
    ) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;

        TrapezoidProfile.Constraints translationConstraints =
            new TrapezoidProfile.Constraints(MAX_CORRECTION_SPEED, MAX_CORRECTION_ACCEL);

        TrapezoidProfile.Constraints rotationConstraints =
            new TrapezoidProfile.Constraints(MAX_CORRECTION_ANG_SPEED, MAX_CORRECTION_ANG_ACCEL);

        xController     = new ProfiledPIDController(TRANSLATION_kP, 0, TRANSLATION_kD, translationConstraints);
        yController     = new ProfiledPIDController(TRANSLATION_kP, 0, TRANSLATION_kD, translationConstraints);
        thetaController = new ProfiledPIDController(ROTATION_kP, 0, 0, rotationConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError(
                "PreciseAlignmentCommand: Failed to load RobotConfig — feedforward disabled. "
                    + e.getMessage(),
                e.getStackTrace()
            );
            robotConfig = null;
        }

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        var state = drivetrain.getState();

        xController.reset(state.Pose.getX(), state.Speeds.vxMetersPerSecond);
        yController.reset(state.Pose.getY(), state.Speeds.vyMetersPerSecond);
        thetaController.reset(
            state.Pose.getRotation().getRadians(),
            state.Speeds.omegaRadiansPerSecond
        );

        prevSpeeds       = new ChassisSpeeds();
        settleStartTime  = -1;
        commandStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        var state = drivetrain.getState();
        Pose2d currentPose      = state.Pose;
        ChassisSpeeds currentSpeeds = state.Speeds;

        // Field-relative PID outputs
        double vxField = xController.calculate(currentPose.getX(), targetPose.getX());
        double vyField = yController.calculate(currentPose.getY(), targetPose.getY());
        double omega   = thetaController.calculate(
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians()
        );

        // Convert to robot-relative for drivetrain
        ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vxField, vyField, omega, currentPose.getRotation()
        );

        // Feedforward — only outside the deadzone to avoid injecting noise while settling
        double[] ffX = new double[4];
        double[] ffY = new double[4];

        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        if (robotConfig != null && distance > FF_DEADZONE_METERS) {
            // F = ma per axis, torque = MOI * angular_accel
            double ax = (targetSpeeds.vxMetersPerSecond     - prevSpeeds.vxMetersPerSecond)     / DT;
            double ay = (targetSpeeds.vyMetersPerSecond     - prevSpeeds.vyMetersPerSecond)     / DT;
            double aa = (targetSpeeds.omegaRadiansPerSecond - prevSpeeds.omegaRadiansPerSecond) / DT;

            ChassisSpeeds chassisForces = new ChassisSpeeds(
                robotConfig.massKG * ax,   // X force (N)
                robotConfig.massKG * ay,   // Y force (N)
                robotConfig.MOI    * aa    // Torque (N*m), passed as omega slot
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

        // Hard timeout — always bail out eventually
        if (now - commandStartTime > MAX_DURATION_SECONDS) {
            DriverStation.reportWarning(
                "PreciseAlignmentCommand timed out after " + MAX_DURATION_SECONDS + "s", false
            );
            return true;
        }

        var state      = drivetrain.getState();
        Pose2d current = state.Pose;
        ChassisSpeeds speeds = state.Speeds;

        double translationError = current.getTranslation().getDistance(targetPose.getTranslation());
        double rotationError    = Math.abs(
            current.getRotation().minus(targetPose.getRotation()).getRadians()
        );
        double linearVelocity   = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double angularVelocity  = Math.abs(speeds.omegaRadiansPerSecond);

        boolean withinTolerance =
            translationError < POSITION_TOLERANCE_METERS  &&
            rotationError    < ROTATION_TOLERANCE_RADIANS &&
            linearVelocity   < LINEAR_VELOCITY_TOLERANCE  &&
            angularVelocity  < ANGULAR_VELOCITY_TOLERANCE;

        if (withinTolerance) {
            if (settleStartTime < 0) settleStartTime = now;
            return (now - settleStartTime) >= SETTLE_TIME_SECONDS;
        } else {
            settleStartTime = -1;
            return false;
        }
    }
}