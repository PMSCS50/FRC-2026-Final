package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.VisionSimSystem;
import frc.robot.Constants.VisionConstants;

public class DistanceBasedShooting extends Command {

    private final Shooter shooter;
    private final VisionSimSystem vision;

    public DistanceBasedShooting(Shooter shooter, CommandSwerveDrivetrain drivetrain, VisionSimSystem vision) {
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooter.rpmControl(vision.getDistanceToPose(VisionConstants.getHubPose()));
        if (shooter.atCorrectRPM(vision.getDistanceToPose(VisionConstants.getHubPose()))) {
            shooter.spinKickers();
        }
    }
    //Should we run sim?
    // I was, there was that error because the robot cant get hubpose at initialization
    //Also 

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted by path planner
    }
}