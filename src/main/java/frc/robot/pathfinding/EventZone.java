package frc.robot.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A zone where the robot activates a certain command
 * Example: activating the robot's intake when going to the neutral zone
 */
public class EventZone extends PathZone {

    private final Command command;

    public OrientationZone(String name,Translation2d min,Translation2d max, Command command) {
        super(name, min, max);
        this.command = command;
    }

    public Command getCommand() {
        return command;
    }

}