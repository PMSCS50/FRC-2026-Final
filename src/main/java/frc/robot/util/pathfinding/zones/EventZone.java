package frc.robot.util.pathfinding.zones;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;

import com.pathplanner.lib.auto.NamedCommands;

/**
 // *A zone where the robot activates a certain NamedCommand
 // ?Example (REBUILT): activating the robot's intake when going to the neutral zone
 * 
 // !EventZones must use NamedCommands rather than just commands
 // !so that our pathfinder can properly retrieve the event markers during log replay
 */
public class EventZone extends PathZone {

    private final Command command;

    //Register a new NamedCommand first then create the EventZone
    public EventZone(String name, Translation2d min, Translation2d max, Command command) {
        super(name, min, max);
        NamedCommands.registerCommand(name + " Event Command", command);
        this.command = command;
    }

    public Command getEvent() {
        return command;
    }

}