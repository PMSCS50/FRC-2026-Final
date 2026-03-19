package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;


public class Intaking extends Command {

    private final Intake intake;
    private final VisionSubsystem vision;
    private Timer intakingTimer;
    private double time;

    public Intaking (Intake intake, VisionSubsystem vision, double time) {
        this.intake = intake;
        this.vision = vision;
        this.time = time;
        
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        this.intakingTimer = new Timer();
        this.intakingTimer.start();
    }

    @Override
    public void execute() {
        intake.spinIntakePID(.8);
        
    }


    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {

        return this.intakingTimer.hasElapsed(time); // runs until interrupted by path planner
    }
} 