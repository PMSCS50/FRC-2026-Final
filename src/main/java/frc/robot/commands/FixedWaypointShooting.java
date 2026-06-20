package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import java.util.Map;

public class FixedWaypointShooting extends Command {

    private final Shooter shooter;

    private final Map<String, Double> distances = Map.of(
        "1. Shooting", 3.628,
        "2. Shooting", 3.31,
        "3. Shooting", 3.021,
        "4. Shooting", 3.272,
        "5. Shooting", 3.593
    );

    private final double distance;

    public FixedWaypointShooting(Shooter shooter, String waypointName) {
        this.shooter = shooter;
        this.distance = distances.get(waypointName);
        addRequirements(shooter);
    }

    @Override
    public void initialize() {}

    @Override
    // |rpmControl logs 2 values: target RPS and actual RPS. atCorrectRPMFixed logs 3 values: current RPM, target RPM, and their difference.
    // |they are separated by topic and subtopic in the logs, so they can be easily compared and analyzed together.
    public void execute() {
        shooter.rpsControl(distance);
        if (shooter.atCorrectRPSFixed(distance)) { 
            shooter.spinKickersMax();
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

//3.628, 3.31, 3.021, 3.272, 3.593