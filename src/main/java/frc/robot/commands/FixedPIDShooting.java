package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;

public class FixedPIDShooting extends Command {

    private final Shooter shooter;
    private double distance;

        public FixedPIDShooting(Shooter shooter, double distance) {
            this.shooter = shooter;
            this.distance = distance;
        addRequirements(shooter);
        
    }

    @Override
    public void initialize() {}

    @Override
    
    public void execute() {

        shooter.rpmControl(distance);
        if (shooter.atCorrectRPMFixed(distance)) {
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