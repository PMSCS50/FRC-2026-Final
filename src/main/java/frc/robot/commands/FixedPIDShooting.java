package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
//import frc.robot.subsystems.vision.VisionSubsystem;

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
    // |rpmControl logs 2 values: target RPS and actual RPS. atCorrectRPMFixed logs 3 values: current RPM, target RPM, and their difference.
    // |they are separated by topic and subtopic in the logs, so they can be easily compared and analyzed together.
    public void execute() {
        // double speed = supplier != null ? supplier.getAsDouble() : distance;
        shooter.rpmControl(distance);
        if (shooter.atCorrectRPMFixed(distance)) { 
            shooter.spinKickersMax();
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