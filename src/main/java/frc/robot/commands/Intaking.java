package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.VisionSubsystem;


public class Intaking extends Command{

    private final Intake intake;
    private final VisionSubsystem vision;

    public Intaking (Intake intake, CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.intake = intake;
        this.vision = vision;
        addRequirements(intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intake.spinIntake(1);
        
    }


    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted by path planner
    }
}