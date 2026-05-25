// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.vision.VisionSubsystem;

// public class DistanceBasedShootingTimed extends Command {

//     private final Shooter shooter;
//     private final VisionSubsystem vision;
//     private int targetId;
//     private Timer shootingTimer;
//     private double time;
    
//     public DistanceBasedShootingTimed(Shooter shooter, VisionSubsystem vision, int targetId, double time) {
//         this.shooter = shooter;
//         this.vision = vision;
//         this.targetId = targetId;
//         this.time = time;
//         addRequirements(shooter);
//     }

//     @Override
//     public void initialize() {
//         shootingTimer = new Timer();
//         shootingTimer.start();
//     }

//     @Override
//     public void execute() {
//         SmartDashboard.putNumber("Distance to tag", vision.getDistance(targetId));
//         SmartDashboard.putNumber("Shooter velocity", shooter.getVelocity());

//         if (vision.hasTarget(targetId)) {
//             double dist = vision.getDistance(targetId);
//             shooter.rpmControl(dist);
//             if (shooter.atCorrectRPM()) {
//                 shooter.spinKickers();
//             }
//         } else {
//             shooter.stop();
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         shooter.stop();
//         shooter.stopKicker();
//     }

//     @Override
//     public boolean isFinished() {
//         return shootingTimer.hasElapsed(time);
//     }
// }