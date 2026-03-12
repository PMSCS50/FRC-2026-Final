package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.VisionSubsystem;

public class DistanceBasedShooting extends Command {

    private final Shooter shooter;
    private final VisionSubsystem vision;

    public DistanceBasedShooting(Shooter shooter, CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooter.rpmControl();
        if (shooter.atCorrectRPM()) {
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