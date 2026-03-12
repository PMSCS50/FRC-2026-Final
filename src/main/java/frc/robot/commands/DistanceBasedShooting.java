package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;

public class DistanceBasedShooting extends Command {

    private final Shooter shooter;
    private final VisionSubsystem vision;
    private int targetId;
    
        public DistanceBasedShooting(Shooter shooter, VisionSubsystem vision, int targetId) {
            this.shooter = shooter;
            this.vision = vision;
            this.targetId = targetId;
        addRequirements(shooter);
        
    }

    @Override
    public void initialize() {}

    @Override
    // public void execute() {
    //     if (vision.hasTargets() && vision.getTargetId() == targetId) {
    //         shooter.rpmControl();
    //         if (shooter.atCorrectRPM()) {
    //             shooter.spinKickers();
    //         }
    //     } else {
    //         shooter.stop(); // lost target — stop spinning
    //     }
    // }
    public void execute() {
        if (vision.hasTarget2(targetId)) {
            shooter.rpmControl(vision.getDistanceToTarget(targetId));
            if (shooter.atCorrectRPM()) {
                shooter.spinKickers();
            }
        } else {
            shooter.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.stopKicker();
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted by path planner
    }
}