// package frc.robot.commands;

// import com.ctre.phoenix6.swerve.SwerveRequest;
// import edu.wpi.first.math.controller.PIDController;
// // import frc.robot.generated.TunerConstants;
// // import edu.wpi.first.wpilibj.Timer;
// // import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// // import com.ctre.phoenix6.swerve.SwerveRequest;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// // import frc.robot.Constants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.VisionSubsystem;
// import frc.robot.subsystems.Shooter;
// // import frc.robot.Constants.ShooterConstants;
// import java.lang.Math;
// import java.util.function.DoubleSupplier;
// import java.util.Optional;


// public class AimAndShoot extends Command {

//     private final CommandSwerveDrivetrain drivetrain;
//     private final VisionSubsystem vision;
//     private final Shooter shooter;

//     private final PIDController rotController;
//     private final DoubleSupplier xInput;
//     private final DoubleSupplier yInput;

//     private double speedLimiter = 0.5;

//     private double vx;
//     private double vy;

//     private final SwerveRequest.RobotCentric drive =
//             new SwerveRequest.RobotCentric();

//     public AimAndShoot(
//             CommandSwerveDrivetrain drivetrain,
//             VisionSubsystem vision,
//             Shooter shooter,
//             DoubleSupplier xInput,
//             DoubleSupplier yInput
//         ) {
//         this.drivetrain = drivetrain;
//         this.vision = vision;
//         this.shooter = shooter;
//         this.xInput = xInput;
//         this.yInput = yInput;

//         rotController = new PIDController(
//                 1, 0, 0
//         );

//         rotController.enableContinuousInput(-Math.PI, Math.PI);

//         addRequirements(drivetrain, shooter);
//     }


//     @Override
//     public void initialize() {      
          
//         rotController.setTolerance(0.07);
//     }

//     @Override
//     public void execute() {
//         vx = xInput.getAsDouble() * speedLimiter;
//         vy = yInput.getAsDouble() * speedLimiter;
//         if (vision.hasTargets()) {
//             PhotonPipelineResult result = vision.getLatestResult();
//             Optional<PhotonTrackedTarget> targetOptional = result.getTargets().stream()
//                             .filter(t -> t.getFiducialId() == 4)
//                             .findFirst();

//             if (targetOptional.isPresent()) {
//                 PhotonTrackedTarget target = targetOptional.get();
    
//                 // 1. Get Distance (Direct 3D vector, ignores height constants)
//                 //This should actually be TargetToShooter btw. we need to mount camera for this.
//                 Translation3d translation = target.getBestCameraToTarget().inverse().getTranslation();
//                 double aprilTagToHub = -0.610816; //negative due to tagspace
                
//                 double dx = translation.getX() + aprilTagToHub; //forward distance to hub (RobotToApriltagX + AprilTagToHubX)
//                 double dy = translation.getY(); //horizontal distance to the hub
                
//                 double distance = Math.hypot(dx,dy);
//                 double phi = Math.toRadians(70);
                
//                 //2. get target and robot yaw
//                 double yaw = Math.atan2(dy,dx);
//                 Rotation2d robotYaw = drivetrain.getPose().getRotation();
                
//                 //3. get field speeds
//                 ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getSpeeds(),robotYaw);

//                 double vxField = fieldSpeeds.vxMetersPerSecond;
//                 double vyField = fieldSpeeds.vyMetersPerSecond;

//                 //3.5 convert fieldSpeeds into tagSpeeds.
//                 double cos = Math.cos(yaw);
//                 double sin = Math.sin(yaw);

//                 double vxTag =  vxField * cos + vyField * sin;
//                 double vyTag = -vxField * sin + vyField * cos;

//                 //4. find and correct values of vv, vy (for shooter, not driving), and yaw
//                 double[] correctedValues = shooter.correctVandYaw(dx,dy,yaw, vxTag, vyTag);
//                 double correctedVx = correctedValues[0];
//                 double correctedVy = correctedValues[1];
//                 double bestYaw = correctedValues[2];

//                 // 5. Control Loop
//                 rotController.setSetpoint(bestYaw);
//                 double rotSpeed = rotController.calculate(robotYaw.getRadians());
//                 drivetrain.setControl(
//                     drive.withVelocityX(vx)
//                     .withVelocityY(vy)
//                     .withRotationalRate(rotSpeed));
    
//                 if (rotController.atSetpoint()) {
//                     double correctedHorizontal = Math.hypot(correctedVx, correctedVy);
//                     double correctedVelocity = correctedHorizontal / Math.cos(phi);

//                     shooter.setVelocityTo(correctedVelocity);
//                     //kickermotors should start when velocity is at right speed but we will do that later
//                     shooter.startKickerMotors();
                    
//                 } else {
//                     shooter.stop();
//                     shooter.stopKickerMotors();
//                 }
//                 return; // Exit early since we found our target
//             } else {
//                 drivetrain.setControl(
//                     drive.withVelocityX(vx)
//                     .withVelocityY(vy)
//                     .withRotationalRate(0)
//                 ); 
//                 shooter.stop();
//             }
//         } else {
//             drivetrain.setControl(
//                 drive
//                 .withVelocityX(vx)
//                 .withVelocityY(vy)
//                 .withRotationalRate(0)
//             );
//             shooter.stop();
//             shooter.stopKickerMotors();
//             return;
//         }

//     }

//     @Override
//     public void end(boolean interrupted) {
//         drivetrain.setControl(
//             drive.withVelocityX(vx)
//                 .withVelocityY(vy)
//                 .withRotationalRate(0)
//         );
//         shooter.stop();
//     }


//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }