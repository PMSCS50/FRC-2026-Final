package frc.robot.util.pathfinding.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BiFunction;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Recovery {

    public static final String NO_RECOVERY = "No Recovery";

    private static final HashMap<String, BiFunction<PathPlannerPath, PathPlannerTrajectoryState, Command>> recoveryCommands = new HashMap<>();
    private static String currentRecoveryMode = NO_RECOVERY;
    private static double minRecoveryDistMeters = 1.5; // Minimum distance in meters to trigger recovery
    private static double recoveryTimeoutSeconds = 5.0; // After this many seconds, recovery gives up
    

    private Recovery() {
        // Private constructor to prevent instantiation
    }

    static {
        recoveryCommands.put(NO_RECOVERY, (path, state) -> Commands.none());
    }

    public static void registerRecoveryMode(String name, BiFunction<PathPlannerPath, PathPlannerTrajectoryState, Command> commandGenerator) {
        recoveryCommands.put(name, commandGenerator);
    }

    public static void setCurrentMode(String name) {
        if (recoveryCommands.containsKey(name)) {
            currentRecoveryMode = name;
        } else {
            throw new IllegalArgumentException("Recovery mode \"" + name + "\" is not registered.");
        }
    }

    public static String getCurrentMode() {
        return currentRecoveryMode;
    }

    public static BiFunction<PathPlannerPath, PathPlannerTrajectoryState, Command> getCommandBuilder(String recoveryType) {
        return recoveryCommands.getOrDefault(recoveryType, (path, state) -> Commands.none());
    }

    public static ArrayList<String> getAllRecoveryModes() {
        return new ArrayList<>(recoveryCommands.keySet());
    }

    public static void setMinRecoveryDist(Distance minDist) {
        minRecoveryDistMeters = minDist.in(Meters);
    }


    public static void setMinRecoveryDistMeters(double minDist) {
        minRecoveryDistMeters = minDist;
    }

    public static double getMinRecoveryDistMeters() {
        return minRecoveryDistMeters;
    }

    public static void setRecoveryTimeout(Time timeout) {
        recoveryTimeoutSeconds = timeout.in(Seconds);
    }

    public static void setRecoveryTimeoutSeconds(double timeout) {
        recoveryTimeoutSeconds = timeout;
    }

    public static double getRecoveryTimeoutSeconds() {
        return recoveryTimeoutSeconds;
    }

}