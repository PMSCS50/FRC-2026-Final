package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;



public class Intaking extends Command {

    private final Intake intake;

    public Intaking (Intake intake) {
        this.intake = intake;
        
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.runPID(.2);
        
    }


    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {

        return false;
    }
} 