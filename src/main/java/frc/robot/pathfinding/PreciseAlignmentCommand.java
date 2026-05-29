// package frc.robot.pathfinding;

// import com.pathplanner.lib.util.DriveFeedforwards;
// import edu.wpi.first.math.controller.HolonomicDriveController;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj2.command.Command;

// import frc.robot.pathfinding.PPLogger;
// import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// //AI Template: Precise Alignment command using a Holonomic Drive Controller.
// //Should be used to correct any translational and rotational pathfinding error.
// //Lowkey I wanna move those RobotContainer drive constants into a new Constants folder to access them here

// public class PreciseAlignmentCommand extends Command {

//     private final CommandSwerveDrivetrain drivetrain;
//     private final Pose2d targetPose;

//     private final HolonomicDriveController controller;

//     // Tune these
//     private static final double TRANSLATION_kP = 4.0;
//     private static final double ROTATION_kP = 8.0;

//     // Tight tolerances
//     private static final double POSITION_TOLERANCE_METERS = 0.015;
//     private static final double ROTATION_TOLERANCE_RADIANS = Math.toRadians(1.0);

//     // Velocity settle requirements
//     private static final double LINEAR_VELOCITY_TOLERANCE = 0.05;
//     private static final double ANGULAR_VELOCITY_TOLERANCE = 0.1;

//     // Translation slowdown radius
//     private static final double SLOWDOWN_RADIUS = 0.35;

//     private final PIDController xController =
//         new PIDController(TRANSLATION_kP, 0, 0);

//     private final PIDController yController =
//         new PIDController(TRANSLATION_kP, 0, 0);

//     private final ProfiledPIDController thetaController =
//         new ProfiledPIDController(
//             ROTATION_kP,
//             0,
//             0,
//             new TrapezoidProfile.Constraints(
//                 Math.PI * 5.0,
//                 Math.PI * 10.0
//             )
//         );

//     // Same request used by AutoBuilder
//     private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
//         new SwerveRequest.ApplyRobotSpeeds();

//     public PreciseAlignmentCommand(
//         CommandSwerveDrivetrain drivetrain,
//         Pose2d targetPose
//     ) {
//         this.drivetrain = drivetrain;
//         this.targetPose = targetPose;

//         thetaController.enableContinuousInput(
//             -Math.PI,
//             Math.PI
//         );

//         controller = new HolonomicDriveController(
//             xController,
//             yController,
//             thetaController
//         );

//         addRequirements(drivetrain);
//     }

//     @Override
//     public void initialize() {

//         thetaController.reset(
//             drivetrain.getState().Pose.getRotation().getRadians(),
//             drivetrain.getState().Speeds.omegaRadiansPerSecond
//         );
//     }

//     @Override
//     public void execute() {

//         Pose2d currentPose = drivetrain.getState().Pose;

//         ChassisSpeeds currentSpeeds = drivetrain.getState().Speeds;

//         double distance =
//             currentPose.getTranslation()
//                 .getDistance(targetPose.getTranslation());

//         /*
//          * Scale translation speed down near target
//          * while allowing aggressive rotational correction.
//          */
//         double translationScale =
//             Math.min(distance / SLOWDOWN_RADIUS, 1.0);

//         ChassisSpeeds targetSpeeds =
//             controller.calculate(
//                 currentPose,
//                 targetPose,
//                 0.0,
//                 targetPose.getRotation()
//             );

//         // Scale ONLY translation
//         targetSpeeds.vxMetersPerSecond *= translationScale;
//         targetSpeeds.vyMetersPerSecond *= translationScale;

//         /*
//          * Feed desired speeds through drivetrain setpoint generator
//          * exactly like AutoBuilder does.
//          */
//         drivetrain.applyAlignmentSpeeds(targetSpeeds);

//         PPLogger.logTargetPose(targetPose);

//         PPLogger.logVelocities(
//             Math.hypot(
//                 currentSpeeds.vxMetersPerSecond,
//                 currentSpeeds.vyMetersPerSecond
//             ),

//             Math.hypot(
//                 targetSpeeds.vxMetersPerSecond,
//                 targetSpeeds.vyMetersPerSecond
//             ),

//             currentSpeeds.omegaRadiansPerSecond,
//             targetSpeeds.omegaRadiansPerSecond
//         );
//     }

//     @Override
//     public void end(boolean interrupted) {

//         drivetrain.setControl(
//             pathApplyRobotSpeeds
//                 .withSpeeds(new ChassisSpeeds())
//                 .withWheelForceFeedforwardsX(new double[4])
//                 .withWheelForceFeedforwardsY(new double[4])
//         );
//     }

//     @Override
//     public boolean isFinished() {

//         Pose2d currentPose = drivetrain.getState().Pose;

//         ChassisSpeeds speeds = drivetrain.getState().Speeds;

//         double translationError =
//             currentPose.getTranslation()
//                 .getDistance(targetPose.getTranslation());

//         double rotationError =
//             Math.abs(
//                 currentPose.getRotation()
//                     .minus(targetPose.getRotation())
//                     .getRadians()
//             );

//         double linearVelocity =
//             Math.hypot(
//                 speeds.vxMetersPerSecond,
//                 speeds.vyMetersPerSecond
//             );

//         double angularVelocity =
//             Math.abs(speeds.omegaRadiansPerSecond);

//         return
//             translationError < POSITION_TOLERANCE_METERS
//             &&
//             rotationError < ROTATION_TOLERANCE_RADIANS
//             &&
//             linearVelocity < LINEAR_VELOCITY_TOLERANCE
//             &&
//             angularVelocity < ANGULAR_VELOCITY_TOLERANCE;
//     }
// }