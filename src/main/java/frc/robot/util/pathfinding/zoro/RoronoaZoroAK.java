package frc.robot.util.pathfinding.zoro;

import com.pathplanner.lib.path.*;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.pathfinding.zones.ZoneManager;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class RoronoaZoroAK implements ShinPathfinder {

    private final RoronoaZoro zoro = new RoronoaZoro();
    private final ZoroIOInputsAutoLogged inputs = new ZoroIOInputsAutoLogged();
    private final String logKey = "RoronoaZoroAK";

    @Override
    public boolean isNewPathAvailable() {
        if (!Logger.hasReplaySource()) {
            inputs.isNewPathAvailable = zoro.isNewPathAvailable();
        }
        Logger.processInputs(logKey, inputs);
        return inputs.isNewPathAvailable;
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        if (!Logger.hasReplaySource()) {
            PathPlannerPath currentPath = zoro.getCurrentPath(constraints, goalEndState);
            if (currentPath != null) {
                logPathData(currentPath);
            } else {
                clearInputs();
            }
        }

        Logger.processInputs(logKey, inputs);
        return reconstructPath(constraints, goalEndState);
    }

    // ─── Logging ───────────────────────────────────────────────────────────

    private void logPathData(PathPlannerPath path) {
        logWaypoints(path.getWaypoints());
        logRotationTargets(path.getRotationTargets());
        logPointTowardsZones(path.getPointTowardsZones());
        logConstraintZones(path.getConstraintZones());
        logEventMarkers(path.getEventMarkers());
    }

    private void logWaypoints(List<Waypoint> waypoints) {
        int size = waypoints.size();
        inputs.numWaypoints = size;
        inputs.waypointAnchors = new Pose2d[size];
        inputs.waypointPrevControls = new Pose2d[size];
        inputs.waypointNextControls = new Pose2d[size];

        for (int i = 0; i < size; i++) {
            Waypoint w = waypoints.get(i);
            inputs.waypointAnchors[i] = new Pose2d(w.anchor(), new Rotation2d());
            inputs.waypointPrevControls[i] = w.prevControl() != null
                    ? new Pose2d(w.prevControl(), new Rotation2d())
                    : new Pose2d();
            inputs.waypointNextControls[i] = w.nextControl() != null
                    ? new Pose2d(w.nextControl(), new Rotation2d())
                    : new Pose2d();
        }
    }

    private void logRotationTargets(List<RotationTarget> targets) {
        int size = targets.size();
        inputs.rotationTargetPositions = new double[size];
        inputs.rotationTargetRotations = new Rotation2d[size];

        for (int i = 0; i < size; i++) {
            inputs.rotationTargetPositions[i] = targets.get(i).position();
            inputs.rotationTargetRotations[i] = targets.get(i).rotation();
        }
    }

    private void logPointTowardsZones(List<PointTowardsZone> zones) {
        int size = zones.size();
        inputs.ptZoneNames = new String[size];
        inputs.ptZoneTargetPositions = new Translation2d[size];
        inputs.ptZoneRotOffsets = new double[size];
        inputs.ptZoneMinPositions = new double[size];
        inputs.ptZoneMaxPositions = new double[size];

        for (int i = 0; i < size; i++) {
            PointTowardsZone z = zones.get(i);
            inputs.ptZoneNames[i] = z.name();
            inputs.ptZoneTargetPositions[i] = z.targetPosition();
            inputs.ptZoneRotOffsets[i] = z.rotationOffset().getRadians();
            inputs.ptZoneMinPositions[i] = z.minPosition();
            inputs.ptZoneMaxPositions[i] = z.maxPosition();
        }
    }

    private void logConstraintZones(List<ConstraintsZone> zones) {
        int size = zones.size();
        inputs.constraintZoneMinPositions = new double[size];
        inputs.constraintZoneMaxPositions = new double[size];
        inputs.constraintZoneMaxVel = new double[size];
        inputs.constraintZoneMaxAngVel = new double[size];
        inputs.constraintZoneMaxAcc = new double[size];
        inputs.constraintZoneMaxAngAcc = new double[size];

        for (int i = 0; i < size; i++) {
            ConstraintsZone z = zones.get(i);
            PathConstraints c = z.constraints();
            inputs.constraintZoneMinPositions[i] = z.minPosition();
            inputs.constraintZoneMaxPositions[i] = z.maxPosition();
            inputs.constraintZoneMaxVel[i] = c.maxVelocityMPS();
            inputs.constraintZoneMaxAngVel[i] = c.maxAngularVelocityRadPerSec();
            inputs.constraintZoneMaxAcc[i] = c.maxAccelerationMPSSq();
            inputs.constraintZoneMaxAngAcc[i] = c.maxAngularAccelerationRadPerSecSq();
        }
    }

    private void logEventMarkers(List<EventMarker> markers) {
        int size = markers.size();
        inputs.eventTriggerNames = new String[size];
        inputs.eventPositions = new double[size];
        inputs.eventEndPositions = new double[size];

        for (int i = 0; i < size; i++) {
            EventMarker m = markers.get(i);
            inputs.eventTriggerNames[i] = m.triggerName();
            inputs.eventPositions[i] = m.position();
            inputs.eventEndPositions[i] = m.endPosition();
        }
    }

    private void clearInputs() {
        inputs.numWaypoints = 0;
        inputs.waypointAnchors = new Pose2d[0];
        inputs.waypointPrevControls = new Pose2d[0];
        inputs.waypointNextControls = new Pose2d[0];

        inputs.rotationTargetPositions = new double[0];
        inputs.rotationTargetRotations = new Rotation2d[0];

        inputs.ptZoneNames = new String[0];
        inputs.ptZoneTargetPositions = new Translation2d[0];
        inputs.ptZoneRotOffsets = new double[0];
        inputs.ptZoneMinPositions = new double[0];
        inputs.ptZoneMaxPositions = new double[0];

        inputs.constraintZoneMinPositions = new double[0];
        inputs.constraintZoneMaxPositions = new double[0];
        inputs.constraintZoneMaxVel = new double[0];
        inputs.constraintZoneMaxAngVel = new double[0];
        inputs.constraintZoneMaxAcc = new double[0];
        inputs.constraintZoneMaxAngAcc = new double[0];

        inputs.eventTriggerNames = new String[0];
        inputs.eventPositions = new double[0];
        inputs.eventEndPositions = new double[0];
    }

    // ─── Reconstruction ─────────────────────────────────────────────────────

    private PathPlannerPath reconstructPath(PathConstraints constraints, GoalEndState goalEndState) {
        if (inputs.numWaypoints <= 0) {
            return null;
        }

        return new PathPlannerPath(
                rebuildWaypoints(),
                rebuildRotationTargets(),
                rebuildPointTowardsZones(),
                rebuildConstraintZones(),
                rebuildEventMarkers(),
                constraints,
                null,
                goalEndState,
                false);
    }

    private List<Waypoint> rebuildWaypoints() {
        List<Waypoint> waypoints = new ArrayList<>();
        for (int i = 0; i < inputs.numWaypoints; i++) {
            Translation2d anchor = inputs.waypointAnchors[i].getTranslation();
            Translation2d prev = extractControlPoint(inputs.waypointPrevControls[i]);
            Translation2d next = extractControlPoint(inputs.waypointNextControls[i]);
            waypoints.add(new Waypoint(prev, anchor, next));
        }
        return waypoints;
    }

    private Translation2d extractControlPoint(Pose2d pose) {
        if (pose == null || pose.getTranslation().getNorm() < 1e-6) {
            return null;
        }
        return pose.getTranslation();
    }

    private List<RotationTarget> rebuildRotationTargets() {
        List<RotationTarget> targets = new ArrayList<>();
        for (int i = 0; i < inputs.rotationTargetPositions.length; i++) {
            targets.add(new RotationTarget(
                    inputs.rotationTargetPositions[i],
                    inputs.rotationTargetRotations[i]));
        }
        return targets;
    }

    private List<PointTowardsZone> rebuildPointTowardsZones() {
        List<PointTowardsZone> zones = new ArrayList<>();
        for (int i = 0; i < inputs.ptZoneNames.length; i++) {
            zones.add(new PointTowardsZone(
                    inputs.ptZoneNames[i],
                    inputs.ptZoneTargetPositions[i],
                    new Rotation2d(inputs.ptZoneRotOffsets[i]),
                    inputs.ptZoneMinPositions[i],
                    inputs.ptZoneMaxPositions[i]));
        }
        return zones;
    }

    private List<ConstraintsZone> rebuildConstraintZones() {
        List<ConstraintsZone> zones = new ArrayList<>();
        for (int i = 0; i < inputs.constraintZoneMinPositions.length; i++) {
            PathConstraints c = new PathConstraints(
                    inputs.constraintZoneMaxVel[i],
                    inputs.constraintZoneMaxAngVel[i],
                    inputs.constraintZoneMaxAcc[i],
                    inputs.constraintZoneMaxAngAcc[i]);
            zones.add(new ConstraintsZone(
                    inputs.constraintZoneMinPositions[i],
                    inputs.constraintZoneMaxPositions[i],
                    c));
        }
        return zones;
    }

    private List<EventMarker> rebuildEventMarkers() {
        List<EventMarker> markers = new ArrayList<>();
        for (int i = 0; i < inputs.eventTriggerNames.length; i++) {
            markers.add(new EventMarker(
                    inputs.eventTriggerNames[i],
                    inputs.eventPositions[i],
                    inputs.eventEndPositions[i],
                    ZoneManager.getEventCommand(inputs.eventTriggerNames[i])));
        }
        return markers;
    }

    // ─── Delegates ──────────────────────────────────────────────────────────

    @Override
    public void setStartPosition(Translation2d startPosition) {
        if (!Logger.hasReplaySource()) {
            zoro.setStartPosition(startPosition);
        }
    }

    @Override
    public void setStartRotation(Rotation2d startRotation) {
        if (!Logger.hasReplaySource()) {
            zoro.setStartRotation(startRotation);
        }
    }

    @Override
    public void setStops(List<Pose2d> stops) {
        if (!Logger.hasReplaySource()) {
            zoro.setStops(stops);
        }
    }

    @Override
    public void setGoalPosition(Translation2d goalPosition) {
        if (!Logger.hasReplaySource()) {
            zoro.setGoalPosition(goalPosition);
        }
    }

    @Override
    public void setDynamicObstacles(
            List<Pair<Translation2d, Translation2d>> obs,
            Translation2d currentRobotPos) {
        if (!Logger.hasReplaySource()) {
            zoro.setDynamicObstacles(obs, currentRobotPos);
        }
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
