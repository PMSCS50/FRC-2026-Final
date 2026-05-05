package frc.robot.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A zone where the robot activates a certain command
 * Example (REBUILT): activating the robot's intake when going to the neutral zone
 */
public class EventZone extends PathZone {

    private final Command command;

    public EventZone(String name,Translation2d min,Translation2d max, String triggerName, Command command) {
        super(name, min, max);
        this.triggerName = triggerName;
        this.command = command;
    }

    public String getTriggerName() {
        return triggerName;
    }

    public Command getCommand() {
        return command;
    }

}