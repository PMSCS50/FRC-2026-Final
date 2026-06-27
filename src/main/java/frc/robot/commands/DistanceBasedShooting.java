package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LLSubsystemMany;
import frc.robot.subsystems.vision.VisionGeneral;
//import frc.robot.subsystems.vision.VisionSubsystem;

public class DistanceBasedShooting extends Command {

    private final Shooter shooter;
    private final VisionGeneral vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public DistanceBasedShooting(Shooter shooter, VisionGeneral vision, CommandSwerveDrivetrain drivetrain) {
        this.shooter = shooter;
        this.vision = vision;
        this.drivetrain = drivetrain;
        addRequirements(shooter, drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.applyRequest(() -> brake);
    }

    @Override
    public void execute() {
        //drivetrain.setControl(brake); // hold position, no scheduling

        double distance;
        if (vision instanceof LLSubsystemMany ll) {
            distance = vision.getDistanceToTarget(ll.getCachedHubPose());
            if (distance < 0) {
                distance = ll.getBestDistanceToHub();
            }
        } else {
            distance = -10;
        }

        if (distance > 0) {
            shooter.rpsControl(distance);
            //if (shooter.atCorrectRPS()) {
                shooter.spinKickersMax();
            //}
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