// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Intake;

// public class IntakeForwards extends Command {
//     private final Intake intake;

//     public IntakeForward(Intake intake) {
//         this.intake = intake;
      
//         addRequirements(intake);
//     }

//     @Override
//     public void initialize() {
//         intake.initinitIntake();
//     }

//     @Override
//     public void execute() {
//         intake.initIntake();
//     }

//     @Override
//     public boolean isFinished() {
//         return !intake.initializing;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         intake.startIntake();
//     }

    
// }