package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Constants.VisionConstants;

import java.util.Arrays;

// 2-camera Limelight subsystem.
// llCamera1: front-facing camera (0 deg yaw offset)
// llCamera2: rear-facing camera  (180 deg yaw offset)
// Fuses pose estimates from both cameras in the drivetrain's Kalman filter.

public class LLSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final String llCamera1; // front camera: peepee peeper
    private final String llCamera2; // rear camera: ass tumor

    private double omegaRps;

    //Pose of the robot, wrapped in latestEstimate
    private Pose2d estimatedRobotPose;
    private PoseEstimate latestEstimate;


    //Vision Sexually Transmitted Diseases
    private static final double BASE_XY_STD_DEV      = 0.5;
    private static final double THETA_STD_DEV        = 9999.0;
    private static final double MAX_AMBIGUITY        = 0.9;
    private static final double MAX_LATENCY_SECONDS  = 0.25;
    private static final double MAX_OMEGA_RPS        = 2.0;
    private static final double FIELD_MAX_X          = 16.5;
    private static final double FIELD_MAX_Y          = 8.5;


    public LLSubsystem(CommandSwerveDrivetrain drivetrain, String llCamera1, String llCamera2) {
        this.drivetrain = drivetrain;
        this.llCamera1  = llCamera1;
        this.llCamera2  = llCamera2;

        setPipeline(9);
        
        setIMUMode(4);
    }


    @Override
    public void periodic() {
        var driveState = drivetrain.getState();

        double headingDeg = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation(llCamera1, headingDeg, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(llCamera2, headingDeg + 180.0, 0, 0, 0, 0, 0);

        PoseEstimate llMeasurement1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llCamera1);
        PoseEstimate llMeasurement2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llCamera2);

        // Reset each loop to ensure we don't accidentally use stale data if both cameras are invalid
        estimatedRobotPose = null;
        latestEstimate     = null;

        boolean cam1Valid = isEstimateValid(llMeasurement1);
        boolean cam2Valid = isEstimateValid(llMeasurement2);

        // Send camera1 pose estimate
        if (cam1Valid) {
            Matrix<N3, N1> stdDevs = calculateStdDevs(llMeasurement1);
            if (stdDevs != null) {
                drivetrain.addVisionMeasurement(
                    llMeasurement1.pose,
                    Utils.fpgaToCurrentTime(llMeasurement1.timestampSeconds),
                    //stdDevs
                );
            }
        }

        // send camera2 pose estimate ,
        if (cam2Valid) {
            Matrix<N3, N1> stdDevs = calculateStdDevs(llMeasurement2);
            if (stdDevs != null) {
                drivetrain.addVisionMeasurement(
                    llMeasurement2.pose,
                    Utils.fpgaToCurrentTime(llMeasurement2.timestampSeconds),
                    //stdDevs
                );
            }
        }

        // Wrap estimatedRobotPose inside a PoseEstimate for more metadata.
        if (cam1Valid || cam2Valid) {
            estimatedRobotPose = drivetrain.getState().Pose;

            Logger.recordOutput("Front Cam PE Odometry PE difference magnitude", Math.hypot(
                llMeasurement1.pose.getX() - drivetrain.getState().Pose.getX(),
                llMeasurement1.pose.getY() - drivetrain.getState().Pose.getY()
            ));

            Logger.recordOutput("Front Cam PE Odometry PE difference magnitude", Math.hypot(
                llMeasurement2.pose.getX() - drivetrain.getState().Pose.getX(),
                llMeasurement2.pose.getY() - drivetrain.getState().Pose.getY()
            ));


            double avgTimestamp = (llMeasurement1.timestampSeconds + llMeasurement2.timestampSeconds) / 2.0;
            double avgLatency = (llMeasurement1.latency + llMeasurement2.latency) / 2.0;
            double avgTagSpan = (llMeasurement1.tagSpan + llMeasurement2.tagSpan) / 2.0;

            // Total unique tags seen across both cameras (no double-counting overlaps)
            RawFiducial[] totalTagsUsed = totalTagsUsed(llMeasurement1, llMeasurement2);
            int totalTags = totalTagsUsed.length;

            latestEstimate = new PoseEstimate(
                estimatedRobotPose,       // KF-fused pose, not raw LL pose
                avgTimestamp,
                avgLatency,
                totalTags,
                avgTagSpan,
                averageTagDistance(totalTagsUsed),
                averageTagArea(totalTagsUsed),
                totalTagsUsed,     
                true // always MegaTag2
            );
        }

        // ── AdvantageKit Logging ───────────────────────────────────────────────

        Logger.recordOutput("Vision/Heading Sent to LL",    headingDeg);
        Logger.recordOutput("Vision/Raw Pigeon Yaw",        drivetrain.getPigeon2().getYaw().getValueAsDouble());
        Logger.recordOutput("Vision/Omega RPS",             omegaRps);
        Logger.recordOutput("Vision/Cam1 Tag Count",        llMeasurement1 != null ? llMeasurement1.tagCount : 0);
        Logger.recordOutput("Vision/Cam2 Tag Count",        llMeasurement2 != null ? llMeasurement2.tagCount : 0);
        Logger.recordOutput("Vision/Cam1 Valid",            cam1Valid);
        Logger.recordOutput("Vision/Cam2 Valid",            cam2Valid);

        if (latestEstimate != null) {
            Logger.recordOutput("Vision/Field X",           latestEstimate.pose.getX());
            Logger.recordOutput("Vision/Field Y",           latestEstimate.pose.getY());
            Logger.recordOutput("Vision/Heading",           latestEstimate.pose.getRotation().getDegrees());
            Logger.recordOutput("Vision/Tag Count",         latestEstimate.tagCount);
            Logger.recordOutput("Vision/Avg Tag Distance",  latestEstimate.avgTagDist);
            Logger.recordOutput("Vision/Distance to Hub",   getDistanceToTarget(VisionConstants.getHubPose()));
            Logger.recordOutput("Vision/Distance to Hub 2", getDistanceToTarget(VisionConstants.getHubPose2()));
        }
    }

    // ── Validation ───────────────────────────────────────────────────────────────

    private boolean isEstimateValid(PoseEstimate estimate) {
        if (estimate == null || estimate.tagCount == 0) return false;

        double ageSeconds = Utils.fpgaToCurrentTime(0)
                          - Utils.fpgaToCurrentTime(estimate.timestampSeconds);
        if (ageSeconds > MAX_LATENCY_SECONDS) return false;

        if (Math.abs(omegaRps) > MAX_OMEGA_RPS) return false;

        Pose2d pose = estimate.pose;
        if (pose.getX() < 0 || pose.getX() > FIELD_MAX_X) return false;
        if (pose.getY() < 0 || pose.getY() > FIELD_MAX_Y) return false;

        return true;
    }

    // Vision Sexually Transmitted Diseases 

    //This one was made by Claude bc aint no way Im creating a system for stdDevs myself
    private Matrix<N3, N1> calculateStdDevs(PoseEstimate estimate) {
        if (estimate == null || estimate.tagCount == 0) return null;

        double xyStdDev = BASE_XY_STD_DEV;

        xyStdDev /= estimate.tagCount;
        xyStdDev *= Math.pow(estimate.avgTagDist, 2);

        if (estimate.rawFiducials != null) {
            double maxAmbiguity = 0;
            for (RawFiducial tag : estimate.rawFiducials) {
                maxAmbiguity = Math.max(maxAmbiguity, tag.ambiguity);
            }
            if (maxAmbiguity > MAX_AMBIGUITY) return null;
            xyStdDev *= (1.0 + maxAmbiguity * 2.0);
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, THETA_STD_DEV);
    }

    // // Tag Overlap and average ambiguity

    // private boolean hasTagOverlap(PoseEstimate est1, PoseEstimate est2) {
    //     if (est1 == null || est2 == null) return false;
    //     if (est1.rawFiducials == null || est2.rawFiducials == null) return false;
    //     for (RawFiducial f1 : est1.rawFiducials) {
    //         for (RawFiducial f2 : est2.rawFiducials) {
    //             if (f1.id == f2.id) return true;
    //         }
    //     }
    //     return false;
    // }

    private RawFiducial[] totalTagsUsed(PoseEstimate est1, PoseEstimate est2) {
        if (est1 == null && est2 == null) return new RawFiducial[0];
        if (est1 != null && est2 == null) return est1.rawFiducials;
        if (est1 == null && est2 != null) return est2.rawFiducials;
        
        RawFiducial[] combined = Arrays.copyOf(est1.rawFiducials, est1.rawFiducials.length + est2.rawFiducials.length);
        System.arraycopy(est2.rawFiducials, 0, combined, est1.rawFiducials.length, est2.rawFiducials.length);

        //Removes all duplicates from the array.
        combined = Arrays.stream(combined).distinct().toArray(RawFiducial[]::new);

        return combined;
    }

    private double averageTagDistance(RawFiducial[] fiducials) {
        if (estimate == null || estimate.rawFiducials == null || estimate.rawFiducials.length == 0) return -1.0;
        double sum = 0;
        for (RawFiducial tag : fiducials) sum += tag.distToRobot;
        return sum / estimate.rawFiducials.length;
    }

    private double averageTagArea(RawFiducial[] fiducials) {
        if (estimate == null || estimate.rawFiducials == null || estimate.rawFiducials.length == 0) return -1.0;
        double sum = 0;
        for (RawFiducial tag : fiducials) sum += tag.ta;
        return sum / estimate.rawFiducials.length;
    }


    private boolean isBetterEstimate(PoseEstimate candidate, PoseEstimate current) {
        if (candidate.tagCount != current.tagCount) {
            return candidate.tagCount > current.tagCount;
        }
        return avgAmbiguity(candidate) < avgAmbiguity(current);
    }

    private double avgAmbiguity(PoseEstimate estimate) {
        if (estimate.rawFiducials == null || estimate.rawFiducials.length == 0) return 1.0;
        double sum = 0;
        for (RawFiducial f : estimate.rawFiducials) sum += f.ambiguity;
        return sum / estimate.rawFiducials.length;
    }

    //getters

    public Pose2d  getPose()        { return estimatedRobotPose; }
    public double  getX()           { return estimatedRobotPose != null ? estimatedRobotPose.getX() : 0.0; }
    public double  getY()           { return estimatedRobotPose != null ? estimatedRobotPose.getY() : 0.0; }
    public double  getYaw()         { return estimatedRobotPose != null ? estimatedRobotPose.getRotation().getDegrees() : 0.0; }
    public int     getTagCount()    { return latestEstimate != null ? latestEstimate.tagCount : 0; }
    public double  getAvgTagDist()  { return latestEstimate != null ? latestEstimate.avgTagDist : 0.0; }
    public double  getAvgTagArea()  { return latestEstimate != null ? latestEstimate.avgTagArea : 0.0; }
    public boolean hasTargets()     { return LimelightHelpers.getTV(llCamera1) || LimelightHelpers.getTV(llCamera1); }

    public boolean hasTarget(int desiredId) {
        return hasFiducial(llCamera1, desiredId) || hasFiducial(llCamera2, desiredId);
    }

    private boolean hasFiducial(String cameraName, int id) {
        return LimelightHelpers.getFiducialID(cameraName) == id;
    }

    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(llCamera1, pipeline);
        LimelightHelpers.setPipelineIndex(llCamera2, pipeline);
    }

    public void setIMUMode(int mode) {
        LimelightHelpers.SetIMUMode(llCamera1, mode);
        LimelightHelpers.SetIMUMode(llCamera2, mode);
    }

    // distance utilities

    public double getDistanceToTarget(Pose2d targetPose) {
        if (!hasTargets()) return -1.0;
        return drivetrain.getState().Pose.getTranslation()
            .getDistance(targetPose.getTranslation());
    }

    /**
     * Returns direct tag distance from rawFiducials of the best camera reading.
     * latestEstimate.rawFiducials always belongs to the camera with better metadata.
     */
    public double getDistanceToTag(int tagId) {
        if (latestEstimate == null || latestEstimate.rawFiducials == null) return -1.0;
        for (RawFiducial fiducial : latestEstimate.rawFiducials) {
            if (fiducial.id == tagId) return fiducial.distToRobot;
        }
        return -1.0;
    }

    public double getBestDistanceToHub() {
        double tagDist = getDistanceToTag(VisionConstants.getMiddleTagId());
        if (tagDist > 0) return tagDist;
        return getDistanceToTarget(VisionConstants.getHubPose2());
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
