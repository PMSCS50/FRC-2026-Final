package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbPush extends Command {
    private final Climb climb;

    public ClimbPush(Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climb.push();
    }

    
    @Override
    public void end(boolean interrupted) {
        climb.stopClimb();
    }

    
}