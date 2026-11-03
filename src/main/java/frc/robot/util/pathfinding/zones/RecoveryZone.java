package frc.robot.util.pathfinding.zones;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.Zone;
import frc.robot.util.pathfinding.util.Recovery;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;

/**
 // *A zone where the robot changes its recovery type when it falls behind
 // ?Application (REBUILT): Changing the recovery type to pathfinding recovery 
 * 
 // !EventZones must use NamedCommands rather than just commands
 // !so that our pathfinder can properly retrieve the event markers during log replay
 */
public class RecoveryZone extends PathZone {

    private final String type;

    //Register an RecoveryZone
    public RecoveryZone(String name, Translation2d min, Translation2d max, String type) {
        super(name, min, max);
        this.type = type;
    }

    public RecoveryZone(String name, Zone zone, String type) {
        super(name, zone);
        this.type = type;
    }

    public String getRecovery() {
        return type;
    }

    @Override
    public List<RotationTarget> createRotationTargets(double entry, double exit) { return new ArrayList<>();}

    @Override
    public List<PointTowardsZone> createPointTowardsZones(double entry, double exit) { return new ArrayList<>();}

    @Override
    public List<ConstraintsZone> createConstraintsZones(double entry, double exit) { return new ArrayList<>();}

    @Override
    public List<EventMarker> createEventMarkers(double entry, double exit) { 
        String previousType = Recovery.getCurrentMode();
        EventMarker em1 = new EventMarker(name, entry, exit, new InstantCommand(() -> Recovery.setCurrentMode(type)));
        EventMarker em2 = new EventMarker(name, exit, new InstantCommand(() -> Recovery.setCurrentMode(previousType)));
        return List.of(em1, em2);
    }

}