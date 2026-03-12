package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;

public class FixedPIDShooting extends Command {

    private final Shooter shooter;
    private double amount;
    private VisionSubsystem vision;
    
        public FixedPIDShooting(Shooter shooter, double amount, VisionSubsystem vision) {
            this.shooter = shooter;
            this.amount = amount;
            this.vision = vision;
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
        shooter.rpmControl(amount);
        if ((Math.abs(shooter.getVelocity() * 60) - Math.abs(vision.rpmFromDistance(amount))) < 50) {
            shooter.spinKickers();
        } else {
            shooter.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted by path planner
    }
}