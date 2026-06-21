package frc.robot.util.pathfinding;

// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * *A zone where the robot activates a certain command
 * ?Example (REBUILT): activating the robot's intake when going to the neutral zone
 */
public class EventZone extends PathZone {

    private final Command command;

    //Create an EventZone with a command
    public EventZone(String name,Translation2d min,Translation2d max, Command command) {
        super(name, min, max);
        this.command = command;
    }

    //Create an EventZone with a registered NamedCommand
    public EventZone(String name,Translation2d min,Translation2d max, String namedCommand) {
        super(name, min, max);
        this.command = NamedCommands.getCommand(namedCommand);
    }

    public Command getEvent() {
        return command;
    }

}