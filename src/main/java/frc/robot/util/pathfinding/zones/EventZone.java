package frc.robot.util.pathfinding.zones;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;

import com.pathplanner.lib.auto.NamedCommands;

/**
 * *A zone where the robot activates a certain NamedCommand
 * ?Example (REBUILT): activating the robot's intake when going to the neutral zone
 * 
 * !EventZones must use NamedCommands rather than just commands
 * !so that our pathfinder can properly retrieve the event markers during log replay
 */
public class EventZone extends PathZone {

    private final Command command;

    //Create an EventZone with a registered NamedCommand
    public EventZone(String name, Translation2d min, Translation2d max, String namedCommand) {
        super(name, min, max);
        this.command = NamedCommands.getCommand(namedCommand);
    }

    //Register a new NamedCommand first then create the EventZone
    public EventZone(String name, Translation2d min, Translation2d max, Command command) {
        this(name, min, max, registerAndReturnName(name + " Command", command));
    }

    private static String registerAndReturnName(String commandName, Command command) {
        NamedCommands.registerCommand(commandName, command);
        return commandName;
    }

    public Command getEvent() {
        return command;
    }

}