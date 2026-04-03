package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;

public class FixedPIDShooting extends Command {

    private final Shooter shooter;
    private double distance;
    // private DoubleSupplier supplier;
   
    // public FixedPIDShooting(Shooter shooter, DoubleSupplier supplier) {
    //     this.shooter = shooter;
    //     this.supplier = supplier;
    //     addRequirements(shooter);


    // }

    public FixedPIDShooting(Shooter shooter, double distance) {
        this.shooter = shooter;
        this.distance = distance;
    }

    

    @Override
    public void initialize() {}

    @Override
    
    public void execute() {

        // double speed = supplier != null ? supplier.getAsDouble() : distance;
        shooter.rpmControl(distance);
        if (shooter.atCorrectRPMFixed(distance)) { 
            shooter.spinKickers();
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