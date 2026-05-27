package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LLSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;

public class DistanceBasedShooting extends Command {

    private final Shooter shooter;
    private final LLSubsystem vision;

    
    public DistanceBasedShooting(Shooter shooter, LLSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter);
    }



    @Override
    public void initialize() {
    }


    
    @Override
    public void execute() {
        double distance = vision.getDistanceToTarget(VisionConstants.getHubPose());
        SmartDashboard.putNumber("Shooter velocity", shooter.getVelocity());
        SmartDashboard.putNumber("Distance to Hub", distance); 

        if (distance > 0) {
            shooter.rpmControl(distance);
            if (shooter.atCorrectRPM()) {
                shooter.spinKickers();
            }
        }
    }




    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.stopKicker();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}