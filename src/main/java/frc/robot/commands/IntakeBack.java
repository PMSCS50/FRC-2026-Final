package frc.robot.commands;

 

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;

 

public class IntakeBack extends Command {

    private final Intake intake;

 

    public IntakeBack(Intake intake) {

        this.intake = intake;

     

        addRequirements(intake);

    }

 

    @Override

    public void initialize() {

      intake.stopIntake();

    }

 

    @Override

    public void execute() {

        if (!intake.initializing) {

          intake.initinitIntake();

        }

        intake.outitIntake();

    }

 

    @Override

    public boolean isFinished() {

        return !intake.initializing;

    }

 

    @Override

    public void end(boolean interrupted) {

        return; //intake has already stopped, nothing needs to happen here

    }

 

   

}