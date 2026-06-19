package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.VisionGeneral;
//import frc.robot.subsystems.vision.VisionSubsystem;

public class DistanceBasedShooting extends Command {

    private final Shooter shooter;
    private final VisionGeneral vision;
    
    public DistanceBasedShooting(Shooter shooter, VisionGeneral vision) {
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter);
        //addRequirements(vision);
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        double distance = vision.getDistanceToTarget(vision.cachedHubPose);

        if (distance > 0) {
            shooter.rpsControl(distance);
            if (shooter.atCorrectRPS()) {
                shooter.spinKickersMax();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.stopKicker();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}