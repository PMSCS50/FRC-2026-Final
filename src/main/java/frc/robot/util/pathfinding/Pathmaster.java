package frc.robot.util.pathfinding;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.util.Elastic;
import frc.robot.util.pathfinding.builders.*;
import frc.robot.util.pathfinding.commands.ShinPathfindingCommand;
import frc.robot.util.pathfinding.telemetry.*;
import frc.robot.util.pathfinding.zones.*;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

// *Command factory that uses PathPlanner's pathfinding features to pathfind to a position on the field.
// *Uses custom pathfinding class RoronoaZoroAK for zone-aware rotation.
// !Must call initializePathfinder() and scheduleWarmup() before using any other methods.
public class Pathmaster {
    private PathConstraints constraints;
    private CommandSwerveDrivetrain drivetrain;
    private Supplier<Pose2d> robotPose;
    private static boolean warmup = false;
    private final LinkedHashMap<String, Pose2d> waypoints = new LinkedHashMap<>();
    private boolean pathing = false;
    private int selectedWaypointIndex;

    // *Constructors
    public Pathmaster(
            CommandSwerveDrivetrain drivetrain,
            double vmax, 
            double amax, 
            double omegamax, 
            double alphamax) {

        this.drivetrain = drivetrain;
        this.constraints = new PathConstraints(vmax, amax, omegamax, alphamax);
        this.robotPose = GoingMerry::getCurrentPose;

        this.selectedWaypointIndex = 0;

        createLoggingCallbacks();
    }

    public Pathmaster(
            CommandSwerveDrivetrain drivetrain,
            double vmax, 
            double amax, 
            double omegamax, 
            double alphamax,
            double nominalVoltageVolts) {

        this.drivetrain = drivetrain;
        this.constraints = new PathConstraints(vmax, amax, omegamax, alphamax, nominalVoltageVolts);
        this.robotPose = GoingMerry::getCurrentPose;

        this.selectedWaypointIndex = 0;

        createLoggingCallbacks();
    }

    // *Call in Robot.java as the last line in Robot contructor
    public static void startWarmupCommand() {
        CommandScheduler.getInstance().schedule(ShinPathfindingCommand.warmupCommand());
        warmup = true;
    }

    private void createLoggingCallbacks() {
        PPLogging.setLogCurrentPoseCallback((pose) -> {
            PPLogger.logCurrentPose(pose);
        });

        PPLogging.setLogTargetPoseCallback((pose) -> {
            PPLogger.logTargetPose(pose);
        });

        PPLogging.setLogStopPosesCallback((poses) -> {
            PPLogger.logStops(poses);
        });

        PPLogging.setLogActivePathCallback((poses) -> {
            PPLogger.logActivePath(poses);
        });

        PPLogging.setLogVelocitiesCallback((linAct, linCom, angAct, angCom) -> {
            PPLogger.logVelocities(linAct, linCom, angAct, angCom);
        });
    }

    // *Register a waypoint on the field. By calling gotoWaypoint() we can align here automatically
    private List<String> waypointKeys = new ArrayList<>();

    public void addWaypoint(String name, Pose2d pose) {
        if (!waypoints.containsKey(name)) waypointKeys.add(name);
        waypoints.put(name, pose);
    }

    public String selectedWaypoint() {
        return waypointKeys.get(selectedWaypointIndex);
    }

    public Pose2d selectedWaypointPose() {
        return waypoints.get(selectedWaypoint());
    }

    public void selectNextWaypoint() {
        int length = waypoints.keySet().size();
        selectedWaypointIndex = (selectedWaypointIndex + 1) % length;
    }

    // !Zone Management
    // *Creates a rotation zone.
    // ?When the robot paths through it, it will rotate to and hold the given heading.
    public void addRotationZone(String name, Translation2d min, Translation2d max, Rotation2d rotation, boolean active) {
        ZoneManager.addZone(new RotationZone(name, min, max, rotation), active);

        Logger.recordOutput("Pathmaster/Rotation Zone " + name, new Pose2d[]{
            new Pose2d(min.getX(), min.getY(), new Rotation2d()),
            new Pose2d(max.getX(), min.getY(), new Rotation2d()),
            new Pose2d(max.getX(), max.getY(), new Rotation2d()),
            new Pose2d(min.getX(), max.getY(), new Rotation2d()),
            new Pose2d(min.getX(), min.getY(), new Rotation2d())
        });
    }

    // *Creates a rotation zone.
    // ?When the robot paths through it, it will rotate to and hold the given heading.
    public void addMultiRotationZone(String name, Translation2d min, Translation2d max, List<Rotation2d> rotations, boolean active) {
        ZoneManager.addZone(new MultiRotationZone(name, min, max, rotations), active);

        Logger.recordOutput("Pathmaster/Multi-Rotation Zone " + name, new Pose2d[]{
            new Pose2d(min.getX(), min.getY(), new Rotation2d()),
            new Pose2d(max.getX(), min.getY(), new Rotation2d()),
            new Pose2d(max.getX(), max.getY(), new Rotation2d()),
            new Pose2d(min.getX(), max.getY(), new Rotation2d()),
            new Pose2d(min.getX(), min.getY(), new Rotation2d())
        });
    }

    // *Creates an orientation zone. 
    // ?When the robot paths through it, it will continuously face the given target pose.
    public void addOrientationZone(String name, Translation2d min, Translation2d max, Pose2d targetPose, boolean active) {
        ZoneManager.addZone(new OrientationZone(name, min, max, targetPose), active);

        Logger.recordOutput("Pathmaster/Orientation Zone " + name, new Pose2d[]{
            new Pose2d(min.getX(), min.getY(), new Rotation2d()),
            new Pose2d(max.getX(), min.getY(), new Rotation2d()),
            new Pose2d(max.getX(), max.getY(), new Rotation2d()),
            new Pose2d(min.getX(), max.getY(), new Rotation2d()),
            new Pose2d(min.getX(), min.getY(), new Rotation2d())
        });
    }

    // *Creates an orientation zone.
    //?When the robot paths through it, it will continuously face the given target pose.
    public void addConstraintZone(String name, Translation2d min, Translation2d max, PathConstraints constraints, boolean active) {
        ZoneManager.addZone(new ConstraintZone(name, min, max, constraints), active);

        Logger.recordOutput("Pathmaster/Constraint Zone " + name, new Pose2d[]{
            new Pose2d(min.getX(), min.getY(), new Rotation2d()),
            new Pose2d(max.getX(), min.getY(), new Rotation2d()),
            new Pose2d(max.getX(), max.getY(), new Rotation2d()),
            new Pose2d(min.getX(), max.getY(), new Rotation2d()),
            new Pose2d(min.getX(), min.getY(), new Rotation2d())
        });
    }

    // *Creates an event zone
     // ?When the robot paths through it, it will schedule the given command.
    public void addEventZone(String name, Translation2d min, Translation2d max, Command command, boolean active) {
        
        ZoneManager.addZone(new EventZone(name, min, max, command), active);

        Logger.recordOutput("Pathmaster/Event Zone " + name, new Pose2d[]{
            new Pose2d(min.getX(), min.getY(), new Rotation2d()),
            new Pose2d(max.getX(), min.getY(), new Rotation2d()),
            new Pose2d(max.getX(), max.getY(), new Rotation2d()),
            new Pose2d(min.getX(), max.getY(), new Rotation2d()),
            new Pose2d(min.getX(), min.getY(), new Rotation2d())
        });
    }

    // *Can make an EventZone out of a NamedCommand
    public void addEventZone(String name, Translation2d min, Translation2d max, String namedcommand, boolean active) {
        
        ZoneManager.addZone(new EventZone(name, min, max, namedcommand), active);

        Logger.recordOutput("Pathmaster/Event Zone " + name, new Pose2d[]{
            new Pose2d(min.getX(), min.getY(), new Rotation2d()),
            new Pose2d(max.getX(), min.getY(), new Rotation2d()),
            new Pose2d(max.getX(), max.getY(), new Rotation2d()),
            new Pose2d(min.getX(), max.getY(), new Rotation2d()),
            new Pose2d(min.getX(), min.getY(), new Rotation2d())
        });
    }

    // *Activates a single zone
    public void activateZone(String name) {
        ZoneManager.setZoneState(name, true);
    }

    // *Activates multiple zones
    public void activateZones(String... names) {
        for (String name : names) ZoneManager.setZoneState(name, true);
    }

    // *Activates only the named zones, but deactivates everything else
    public void activateOnly(String... names) {
        ZoneManager.setAllZones(false);
        for (String name : names) ZoneManager.setZoneState(name, true);
    }

    // *Deactivates a single zone
    public void deactivateZone(String name) {
        ZoneManager.setZoneState(name, false);
    }

    // *Deactivates multiple zones
    public void deactivateZones(String... names) {
        for (String name : names) ZoneManager.setZoneState(name, false);
    }

    /** Deactivates only the named zones, but activates everything else. */
    public void deactivateOnly(String... names) {
        ZoneManager.setAllZones(true);
        for (String name : names) ZoneManager.setZoneState(name, false);
    }
    
    // !Pathfinding Commands
    // *Pathfind to any field pose with obstacle avoidance
    public Command makePathTo(Pose2d destination) {
        if (!GoingMerry.isConfigured()) return Commands.none();
        pathing = true;
        return Commands.defer(
            () -> GoingMerry.pathfindToPose(destination, constraints),
            Set.of(drivetrain)
        )
        .finallyDo(() -> pathing = false);
    }

    // *Pathfind to any field pose with obstacle avoidance
    public Command makePathTo(Pose2d destination, List<Pose2d> stops) {
        if (!GoingMerry.isConfigured()) return Commands.none();
        pathing = true;
        return Commands.defer(
            () -> GoingMerry.pathfindToPose(destination, stops, constraints),
            Set.of(drivetrain)
        )
        .finallyDo(() -> pathing = false);
    }

    // *Pathfind to a registered waypoint
    // ?Waypoints are defined in Robot.java and updated with alliance-relative poses in robotPeriodic()
    public Command gotoWaypoint(String name) {
        if (!GoingMerry.isConfigured() || !waypoints.containsKey(name)) return Commands.none();
        pathing = true;
        return Commands.defer(
            () -> GoingMerry.pathfindToPose(waypoints.get(name), constraints),
            Set.of(drivetrain)
        )
        .finallyDo(() -> pathing = false);
    }

    // *Pathfind to a registered waypoint
    // ?Waypoints are defined in Robot.java and updated with alliance-relative poses in robotPeriodic()
    public Command gotoWaypoint(String name, List<Pose2d> stops) {
        if (!GoingMerry.isConfigured() || !waypoints.containsKey(name)) return Commands.none();
        pathing = true;
        return Commands.defer(
            () -> GoingMerry.pathfindToPose(waypoints.get(name), stops, constraints),
            Set.of(drivetrain)
        )
        .finallyDo(() -> pathing = false);
    }

    // *Pathfind to waypoint corresponding with selectedWaypointIndex
    public Command goToSelectedWaypoint() {
        if (!GoingMerry.isConfigured()) return Commands.none();
        pathing = true;
        return GoingMerry.pathfindToPose(
            waypoints.get(waypointKeys.get(selectedWaypointIndex)), constraints
        )
        .finallyDo(() -> pathing = false);
    }

    // *Pathfind to waypoint corresponding with selectedWaypointIndex
    public Command goToSelectedWaypoint(List<Pose2d> stops) {
        if (!GoingMerry.isConfigured()) return Commands.none();
        pathing = true;
        return GoingMerry.pathfindToPose(
            waypoints.get(waypointKeys.get(selectedWaypointIndex)), stops, constraints
        )
        .finallyDo(() -> pathing = false);
    }

    // *Intended alignment pipeline.
    // ?pathfindToPose() has ~5cm error at endpoint.
    // ?A predetermined .path file has much less error, around <1cm.
    // ?This pathfinds to the start of the .path, then follows it precisely to the end.
    public Command pathFindThenFollowPath(String pathName) {
        if (!GoingMerry.isConfigured()) return Commands.none();
        try {
            pathing = true;
            PathPlannerPath path;
            if (pathName.startsWith("choreo/")) {
                path = PathPlannerPath.fromChoreoTrajectory(pathName.substring(7));
            } else {
                path = PathPlannerPath.fromPathFile(pathName);
            }
            return Commands.defer(
                () -> GoingMerry.pathfindThenFollowPath(path, constraints),
                Set.of(drivetrain)
            )
            .finallyDo(() -> pathing = false);
        } catch (Exception e) {
            pathing = false;
            Elastic.sendNotification(
                new Elastic.Notification().
                withLevel(Elastic.NotificationLevel.ERROR)
                .withTitle("Pathmaster Error")
                .withDescription( "Path " + pathName + " is not defined"));

            return Commands.none();
        }
    }

    // *Intended alignment pipeline.
    // ?pathfindToPose() has ~5cm error at endpoint.
    // ?A predetermined .path file has much less error, around <1cm.
    // ?This pathfinds to the start of the .path, then follows it precisely to the end.
    public Command pathFindThenFollowPath(String pathName, List<Pose2d> stops) {
        if (!GoingMerry.isConfigured()) return Commands.none();
        try {
            pathing = true;
            PathPlannerPath path;
            if (pathName.startsWith("choreo/")) {
                path = PathPlannerPath.fromChoreoTrajectory(pathName.substring(7));
            } else {
                path = PathPlannerPath.fromPathFile(pathName);
            }
            return Commands.defer(
                () -> GoingMerry.pathfindThenFollowPath(path, stops, constraints),
                Set.of(drivetrain)
            )
            .finallyDo(() -> pathing = false);
        } catch (Exception e) {
            pathing = false;
            Elastic.sendNotification(
                new Elastic.Notification().
                withLevel(Elastic.NotificationLevel.ERROR)
                .withTitle("Pathmaster Error")
                .withDescription( "Path " + pathName + " is not defined"));

            return Commands.none();
        }
    }

    //*Pathfinds to the nearest pose from a list of candidates.
    // ?Copied from Spartronics.
    public Command pathToNearestPose(List<Pose2d> candidates) {
        if (candidates.isEmpty()) return Commands.none();
        pathing = true;
        return Commands.defer(
            () -> {
                Pose2d nearest = candidates.stream()
                    .min(Comparator.comparingDouble(
                        p -> p.getTranslation()
                            .getDistance(robotPose.get().getTranslation())
                    ))
                    .orElseThrow();
                return GoingMerry.pathfindToPose(nearest, constraints);
            }, Set.of(drivetrain)
        )
        .finallyDo(() -> pathing = false);
    }

    // *Pathfinds to the nearest registered waypoint.
    public Command pathToNearestWaypoint() {
        if (waypoints.isEmpty()) return Commands.none();
        pathing = true;
        return pathToNearestPose(waypoints.values().stream().toList())
        .finallyDo(() -> pathing = false);
    }


    /**
     * *Pathfinds to a destination while arriving faced toward a separate target.
     */
    public Command pathfindFaceTargetPose(Pose2d destination, Pose2d faceTarget) {
        pathing = true;
        return Commands.defer(
            () -> {
                Rotation2d facing = getRotationToPose(destination, faceTarget);
                Pose2d oriented = new Pose2d(destination.getTranslation(), facing);
                return GoingMerry.pathfindToPose(oriented, constraints);
            }, Set.of(drivetrain)
        )
        .finallyDo(() -> pathing = false);
    }

    /**
     * *Pathfinds to a destination while arriving faced toward a separate target.
     */
    public Command pathfindFaceTargetPose(Pose2d destination, Pose2d faceTarget, List<Pose2d> stops) {
        pathing = true;
        return Commands.defer(
            () -> {
                Rotation2d facing = getRotationToPose(destination, faceTarget);
                Pose2d oriented = new Pose2d(destination.getTranslation(), facing);
                return GoingMerry.pathfindToPose(oriented, stops, constraints);
            }, Set.of(drivetrain)
        )
        .finallyDo(() -> pathing = false);
    }

    // *Runs a PathRequest as a ShinPathfindingCommand or a PathPlannerAuto, depending on what you want
    // *Best for more complex pathfinding routines or if you are stupid enough to use this library for auton routines
    public Command submitRequest(PathRequest request) {
        pathing = true;
        return Commands.defer(() -> {
            if (request.getActiveZones().length > 0) {
                activateOnly(request.getActiveZones());
            }
            if (request.runsAsAuto()) {
                //Creates a PathPlannerAuto out of the ShinPathfindingCommand, and fills in added event triggers
                return request.EventTriggerFunction().apply(
                    new PathPlannerAuto(GoingMerry.buildRequest(request, constraints), robotPose.get())
                );
            }
            return GoingMerry.buildRequest(request, constraints);
        }, Set.of(drivetrain))
        .finallyDo(() -> {
            pathing = false;
            ZoneManager.setAllZones(true);
        });
    }

    /**
     * *Cancels any currently running pathfinding command. Not needed for now
     */
    public Command cancelPathing() {
        return Commands.runOnce(() -> {
            Command current = drivetrain.getCurrentCommand();
            if (current != null) {
                if (current instanceof ShinPathfindingCommand || current instanceof PathPlannerAuto) {
                    current.cancel();
                    pathing = false;
                }
            }
        });
    }

    //Log Stuff
    public void log() {
        Logger.recordOutput("Pathmaster/Selected Waypoint", selectedWaypoint());
        Logger.recordOutput("Pathmaster/Selected Waypoint Pose", selectedWaypointPose());
        Logger.recordOutput("Pathmaster/pathing", pathing);
    }

    // !Helpers
    //* Returns the rotation needed at 'from' to face toward 'target'
    private Rotation2d getRotationToPose(Pose2d from, Pose2d target) {
        Translation2d delta = target.getTranslation().minus(from.getTranslation());
        return new Rotation2d(delta.getX(), delta.getY());
    }

    public boolean isPathing() {
        return pathing;
    }

    public boolean warmedUp() {
        return warmup;
    }
    
    public boolean GoingMerryPathFindingConfigured() {
        return GoingMerry.isPathfindingConfigured();
    }

    public boolean GoingMerryConfigured() {
        return GoingMerry.isConfigured();
    }

    public Pose2d[] getActivePath() {
        return PPLogger.getActivePath();
    }

}




/*

#+- ..    . .*%@@@@@%%%##*.+****+:   .....::-..:::::.   .                                     ...:::-------::....  .......::---=+++**##%%%%%%*=+++===#
::..  .  .  ..    .     .. :-----.     ....:. ---:::: . ..                                             ...        .:::--===+++******##*++++*##**###**#
*=-::..:..  .. =.    ...   ......:      ..:...===-+++=-..                                  .:-:-:----:::...       ::--===++++++********###########%#%#
%%%%###+=-:...  +-:...:.  .====-+=:..:--==+:-*******=         .::-:-=====-=======-::.        :**#*******+=--::::..+*################*#########%%%@@@@#
%%%%%%%###*+#*####%##*=...-+++.:--..::-==++-+***+***-         .:..::::::::......            .=+***=**#*++==-:::...+*#%%%%###%%%%@@@@@@@@@@@@@@@@@@@@%#
@@@@@@@%%%%%%%%%%%%#*+..:.:+-...:..::-==++*+*#**+*****=-:                             ..:-=++++***-+****+=--:::::.=*#%%%#*:-*#%%%%@@@@@@@@@@@@@@@@@@@#
%@@@@@@@%%%%%%@@@%%#*=...:.:.......::--=++******+*+******+++++=-   =-:=-:--::----======++++++****+:--***++=-:::..::+*###**:.=*#%%%@%%@@@@@@@@@@@@@@@@#
@@@@@@%@@@@@@@@%%%#*+...::...... ..:--=+++++++++=+++++++======-:   .-= :#==--*+==-==---===========:..+===---::.....:+****=...**##%%%@@@@@@@@@@@@@@@@@#
@@%@%%%@%%%%%@@%##*+. ::::.:......::-=++****#*****++++==+=++==-     .+=. #-.=# ========--======++--::=++++=--::.... =+++-.....*#%%%%%%%%%%%%@@@@@@@@@#
@@@@@@@@%%%%%%#**+-. .::::::.::..:::-==+*+***+******+*++++***=    .  -*=-.= .= .-=--===---===-=--:-:--==+++=-::......+++- ....-+**####+=##%%%%%%@@@@@#
%%%%%%@%%##%%#*##*= :-:-::::..  ...::-===-::::.-+===--=:..   .   .  :=++*...    .: -.  *:-..... :::==-:.-=+=--::.....-++..:::..*#%%%%%%##%##%@@@@@@@%#
@@@%@%####%%%%###*..:::::.    : ...::::=:.::::. .:.::: .=.  :    .   =-#+#:.-   ...= .- .  .... .--=:.. .===--::..    :.  ...:-+*##%%%%%%%%%%%@%%%@%%#
%###%%%%%#######*= .:::.          ..:---:..:.:....:::...   .    .-    -:#@#..    .. - . =         . .:::..:-::.          ....  .*###%%%%%%%@@%%%%%%%%#
#%%%%########*##*+.                 ..    :::...   .+:.-.     .      ..-=+@@     ...-:  ::        . . .....:...           ...   =**#%%%%%@%%%%%%@@@%##
%%%%%%%%###**++=+=.                   ...  .      *+..- .-=.  - :.      +#=      ..:*:  .+       ........:...:....         ... . **###%%%%%%%%@@%%%%%#
##**===**###**#*=-                 ...::::::::++*#=-=*   + :  +.+ -   =.=+ .  .  . := .  *=-:-:-=-::...::-::.:::....             -++-:-*######%%%%####
*:       :+=---:..          .....::::-::--*+++*#-+===+.  + +  +-      =.-=     . .-:= . .-==-=+**+++=-----=--.:::::..   : .....   :...:::..  .+#+. ..#
=..:-:      :...:::        .:..:---==++++*++##*=---:-    :  = .  . *     :.    .-.*-=- .:*:=+++=+++*++*****++=+++=-:....  .....   :.-::::.     -: ...#
 ...::......:.  .::.  .- ....::...:-=+++#*##%=-=+:+**- .  . :=-  - :* -  .-  :  -#=#=+. =@* -++=======+=---=++++==---:--:::..     ::..::.     ......:#
.......  .......-==.   .   .::---=-*=+++###=*+***#* -+  :   .+    ..*+  - : .-  :=@@==.#%: -+-.::-:==-=+-------:=:...:::::::::.. .:::...      ..::.. #
:.  ...:-+-::=+=+++:   ..  . ....:=+:-=+*#=:-:=+++++ :+  . -      . +   . - .   -=#@* *   -*= -:-====-======-:.::::...::::::::.. -==++:      .:---.  #
.   :*#**+=+-:::----. .::  ....:: --===:#+=*---++-=-=  = -  . -   :+   .  + :   :-%=-   .-=--==.****+*++===*+-...:........:..    ---==++*+**+=:::... #
###+------. -=----=-- ::-. .....:--=+*:%+=+++-...=++ :    :  :  .  =   :  -.    *@@:. .:.**+-.+**+*=+**+++*****++=-.       .... .+++=:-=---===+=+++..#
+*+=-=++++==*:. .:..:. ..:  ..::::-++*##--+...: .:::+  :  :-   + :-- . .. =:   +%@.. +*+=*= +=++++++++=---===+++=---::.   ..     ==++***+**+=. ..:.::#
****-:=::... :+#%%####-**++ :---::-++**+=* : .=    . -      - -     =   = =:::@@@@@+*++.. ++#+**+++++==-=------=---::.. .:.... -.:--:===--::--*+***-.#
:.:-.-+*####%@%%%%%##**+-+= ..::::=+*++=*.....-+=::-: + .            :  +  :  @*-@@-#+-**+@:@--=----=-----::--::-::::..  -:    ===*#%%%*:-=:--+=++***#
###%%%#**+++===***++---- = .:::::-=+*+=-+:.-=*-   .-*-- =  -.-        :=.==    *=@@#++#%+*#*#%:::-=:-===+-=-=+--.. ...   ---   =-=**++--+#*++-. ..:-=#
*#*####%%*==******+++==  .   ..::-=***+-+-+:        ..%-+      .      ::::=%@   =#@+@@:%+%:***+#=:-:---=:::::-:--::.    .-:.  .:-=-:-=:-****#+=****:.#
%%%%%####*****+++.===:     ...:::-+#*#*-*+: .       .:=+*==.+-: .:   -:...:==%..#@#@:%.+*=---*.@@:-=+++====++====-..  . ----::::.:---==:::===--.*#***#
+*##%%%#**++++=.: .::           .-*######%         :.::=++*+-=% ::.:  -:..#.=@: .@-@*+*+*=+*%*@%+-=*#=*==--:++==-:-:::   :::==:== ::--::::-:::: :==++#
#####**#*++==:-            :--==+**##%%+%+        :=    :..%# %:    -::.. .=:  .@#%%.%*=---*----*-:=+%+-*=-+*+-=---:::.. :+=+:.....--=-:::::.    ::-:#
#**==+*-::.- :        .....:-:--:-+##%%##*        .      ..:=@ -  :..:.==++++=*=@+=%=-+--+   .:--+--+*++*:::-===-=--::... :.==++*. . .::::.       ---#
--*#****+==     .. ......:::---=:+#*##%#..:               .:.=%-*        *=:==-@===::: -   ...:-+*+***+++-+-:-....::.....  . -++- . . =-:     .   :::#
-#%%%*+++=:  . ......::.:::--==-:+#*###%* .       :       .:.==*-*   : = # -==%-@::.: :: :.:::-*#=-+%.-:++*=-:-::........    ::  .. ...--....     .:-#
*#*++*#*-.    .:.:::....:::----=+++#%#=+= =        =.:     .+.-:.*+   +==-+*+*--@:..:-   : --*.   :.:*%%%%%*..::--:::.                 ..  .:::..::::#
***+=---- .....  .....:::-------+%#+=-+==%=  -           :.: ..:==:= .    :=%*:@#-. .= ..::##      .::#%#=%+..    .::...        ...         .... . :-#
*==:--:..      ....:::::::-==+=+-+##*-+++*+=   -    :    :- =%%*-=.    = ++  -** :..+.:::-%:#      ..:++*%%-:::.....:.                              :#
+=+*+=-:    . .:::---::---------:-=: ...###*  :-   :+.. .:-:--#%%%-     =  ..:*...--- ..*@.:       :.:* +#%==----..                    :::: .      .:#
+**+-:::        ..::-::::...-----:..     *%#      ++.+ :-:::++@:#%*..  :=  ..*.::+#@-:.+      .   :..=  *#:----=-:.                   .--:. .      :-#
=-=====-           .. ...  .--==+=::  .  :@*:  :  .:  -    :=::::.=+=:.:= :.-+ :-@@%#=     :.:.  .:.+:=*%=:=--===-.......    .        :::    .    .--#
-.-==--=          ...:::---.   -:.: ==+:%@#:     =.        ... =    +:    * %@:-@%%@@     ...:= :===#*+@.%%%.::-=-:::.:...           .:- .:.     .=--#
- .-:--:                .::.        . -=#%%-+-  : :        .          = .:--+@#@%%%@     :..-=@   :::=:@@%= :.====..:::...           ....:::...  .:..#
:  =--.             .:---=:::.........   *+%*:+ :-#::.=:  @   :    .   ::.:-*=#@%%@-     :.==%  .   .=#%@      --=---::..               .::...:=.+++=#
.    :.::     .       :--.:.:.:.....  .   :@*#+=-- #:.   *   @      -:- ..--:-:%%@@- .-  ::-@-   -  .=*%:       ::---:...          .   ...... .-==***#
...  ::::   . :        .:..::::::......   -*%@-=.-  :    -  =:     ..- :=-- .   ..        -@    .=:-=*%*          ......           ......... ..===+**#
 ... .::: . ..:        ....::::::.........:##:  .    -   .  -:- ...::+      : :==   -      .-   ::-%@**:         .. . .                 ...... ===++*#
.... :.:.::.::: :       ..::...............+:-  :=--+..     ::.=.:+      .  . :::  *+ + ..::    +.==@.               ....                      -=====#
 .   ::.::..:::::.      ...:::.:::.....:      :=.= =.:-..=.:-:+:- .       :*#  ::+:++++#=      .::=+#                ...              . :::... .+*-:-#
... ::. ::::..:: .        .:.::......-         ::=:-. :*%=::::.+-:    ....:-   - :+####%      .+-=@+                 . ...      .      .::.... .++***#
. . -:   .::::::         ....:::....:             :.  # .#-:-#:   + ....-:     -.=.==+=+=:...=--@=@ :  . .....   . .. . ..::.... .        .   ..**##*#
.  ::-.   .:.:..         ..:::::.:.  .             ..:  + =:.:      .:+. .:  ..   .=.+##@*+#%@@@@=@@ :.....:.......... .............            ::   #
  ::--     :::           .....:::=      .           .:=*+ .  -.:::.. :.:-    .+:-  -.+-#@@@@@@@@ *=+@.- :.::.....:::::. ....   ...... .        ...   #
..-::.      .:           .:---:-:       .     .       ......  =.::-.::-   =::+* -:+:-::+@@@@% =-@#@*%--.:-==:.-:=-.............  ......       ..    .#
:::.:                 .:::-=---                  ........ =   -          +    :.*-.=+:+@@@@% +-+--#:*%-.:=-.:--=-....-::.::.:.....           :::    .#
.....                ::---=-= .             .   .... .... *=- - =..  -+::- =  .-=@@%-+*=-=. ....-=---=:.::::---:-:-=-===-::::::..... .       :-:::: .#
.   .     .         .::--=---+     .          =....-#@@@@*-::  .:+  :   :+#..+.:+ =          .-....==--::::.:::::::::.---==::::.::.:..       . --:   #
  .      ...         .:::::::::   = .  .    ..... -:=++=+=- =-::+:=- - .- : #=-: =         -#   -+..-#:*. :::::...:....:::::::--::::::.        -::   #
.        ..........      .:..: :   .=+: ......:::=---=#+@@@% =***:*=::-.++++             =:     ....-.%@=   .::....:-=-:--...::-:--::..         -    #
        ...:........:---:::.-- .                 ==-+##-*@@@#+@:   =+:-  @%.+          .+        .   *.%:  . ......:::----====------:...        .    #
       ...::.... .--==++=+====  *  ...        .--+%-:=+#@*--%+:......   .*.+ -   -    :               #=#:  .....:::==--:-----=-==--:..              #
     ..........-::-=--=--=++++=+   ::::==-    .-=#@#%= --# =::::..:....  :  - :       :                @@+.     ...::-:---=----:    .                #
        ....-:-::---:..=+====-====+     .....+=-:=.=-#=##@@-:::.. ...... .            .                =@=*...    .  ::::::: .::-==....              #
      -----:-:---=-+=+==-:=++=------:*===--=%@.-.=+@:*+:...:..:..         :      .                     :@==:  .       ...:::::---=--=-::..        .  #
     .::----===---+=-====***-==+=====---@@@*  =:**@#--. .:.:....      .     -      :                    #=-=::::.:.:--......:::::::...:.:.         . #
........::===--=+++**++++===---==----------#-+*-#=@=-.. :::::::..     .       #      .                  ---*..::::----:----:.       .:---::          #
......::--==::=*+++====+++=-:----------:::.-+::-=+:.:..  ... ..             =  -=+      =              -%#-=....:::::..:--==-----:. ..:::::..        #
.::-=+==+==--=::::::----:..::::::---::::..:..  :.:..::.. ...  .                    +                 .=@@-=+:....:::.:----=======--=-::====--::      #
:::---.:..:.===----:..:::-:-::.:::::::::...  .::::::...: .....                .-:   @:=:           ..*@@#-##@:.:::...:::::--------===+==-..::...     #
:.  .:-----==-=---:..:::--:-:::::::.....           .              .             ::+  = *..-        .@@-@ ==--=..:.:.::.::-==::-::-:.::::.......:.    #
    .::--==--:===- .::=-:------::.... .  . ..::....  .    .        .           .  =- .   -        =@@:= .%* -=- ... :.   .:::--:::::=+#+=:++---.. . .#
        :=--=---.....:-==------......  .  ..       ..   .                      .   := .    .     %=: + ..@@  +@=.. ...  ....::-:::-----======--......#
            .::. .  --::-::::::.....                                           . : .                +  .-=%  :+@:..:.    ...::::--===========++.  ...#
                   ...              ........... ....                           - .        ..       =  . . :  ..=*   .    :-:-=========+++=+=-=++. .  #
                  --:-::.      ..........:.::::::...  .                        .  =     .         =     -.=  .= =% ..     .--:::--==+++++==++-.... ..#
        .:.: :.  --.::....::--::.......... ....:......... .. .                     =     :      ..        :   -+@- ..  ....--=--==+---:=+=+++-....   #
       ...:.:   ==:.::::::::::::.......... .  ......  ...... ...                  .  =                 :  . .--@=::..::.::---===+====-=-=+++++*+=:   #
      .:. .-.  --: .::.::::::::::.....:..... ....:..........    ..             -    .  *:      :.        . -=*@:..:::.....::--====--------+++++=.    #
       ..:-   :::  :::.::=--:::-:................... .                          .-         .-.     -.--.-..+#@-.  ......:::::--:---::::::: :==+++.   #
       .--.. ::.  .::::=------:::::::......::. .:.....    ... .... ..              .=--:. .... ....  . ...:@@+.  ......::--------=======-:: .:-:=..  #
            .::   :.:-:-----=--:.:.:-..:::.  .:.....      .................           =-.....  ------=:. *@@:. ...::::::::::----=+--=----:- .:.:::.  #
             :...-++==-==:---=--- .:----...:..:::......   ........ ....      ..         ---..--+#@%+- -%@#=-.......:::--------:::-==--==--::-.  ...  #
                    -===--:---=---- :.:..::............ .  ..   .  ....                   -..--*-----#*@@@@*  ....:.:::-==--=-==---.         :   ..  #
                  .--===+=====-=--:    .          ..................               .            ##*%#-%*%@#-  ..::::----========.             .      #
                  .=++++++===:--:..:---:::::---:::...          ...    .              .....       :++*%**@@@:...:::::--: .----==       ..:.....       #
               ... :=+++=-:..:-======-=----:--:::::....-....   .........              . .......  =+%*++*#@%%.:.::::.::--    .-.    .. ..::... ..     #
        . .  .     ..:-::-==-----====-=---:----::::-.:.....   :..........    ........... ........ %+-:=%*#@@: .::::-:---:---.        .. .....   .    #
            .:.. ....-+==.+======----:-:-------------:::..   ::.......................:::.....::.:=+#*+**#*@*--  ::::-----::-:     . ......      .   #
         ............. -++++=+====- --::-::-=::: -:::::.:   .:...::..........::.:::::-:::::::::::--@#%%**-#=@*-::  :.::::---- .      ... ..     . .. #
        ...... ... ...=++++=====- --=-::. ....             .::..:.::.::::::...:::::::::::::::::::-:*##+-++:-+%+-=-  .-:.-====::  ...             ..  #
             ..       :*++==++ .-::: ....  .               .:.::..::::::::::--::::::------------:::=%++ :++=:#*=.-=   .=  .-==+-. ...             .. #
                     .:--=-  ==-.       ..                .---:: :::::..:.::---:::-:::-:--------==--@*=:  =-+-@:-::     .-  ..                       #
                           .+=+=-     .....         ..    :-:::        ...:--:::-:::--------------==+%+-. .. :=#+:-:                .                #
                           --::-: . .....:...        ....::::.         .....::-::::-::--:-.----------**:=  .=:.**.:::                         . .    #
        ...                . *+=::   .......      .....:-:....            ....::-:-------- --------:::=--   =:-.#:+---.                         .    #
  . ... ...                 .:....                   ..:: ..                ...:.::-------. =-=-- .....: :   +.:-*-++**....                          #
... ......                         .                       .                   .:---:-----: .-. .   . .  :.: ===.+. .......             . ....... .  #
:........                                                                       .-=====-. - . .            :::- = .                   .... :.... .=..#
**..=:..                             ..  .             .                         .:-++++==:                 . . .... . .     . ..          .%%%%%%%%%#
-+##:.                                 .                                          ..:-..                     . ...               . .. ..    .*%%%%%%%#
###=..                                                                            .. .    .                        . .               ..         %%%%%#
*#....             .                                                              .  ..                                                          #####
######################################################################################################################################################

*/
