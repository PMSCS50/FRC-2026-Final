package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbPull extends Command {
    private final Climb climb;

    public ClimbPull(Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    
    @Override
    public void end(boolean interrupted) {
        climb.stopClimb();
    }

    
}