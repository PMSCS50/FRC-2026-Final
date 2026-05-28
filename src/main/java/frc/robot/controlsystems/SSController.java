package frc.robot.controlsystems;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Generic state space controller.
 * Given the system, handles the full LQR + Kalman + feedforward loop
 * for any single-input mechanism.
 *
 * Usage:
 *   1. Instantiate with a given LinearSystem and tuning parameters
 *   2. Call setSetpoint() to set desired state
 *   3. Call calculate() every loop with current measured outputs
 *   4. Apply the returned voltage to your motor
 * 
 * Im trying to add more comments to make our code more readable
 */
public class SSController<States extends Num, Inputs extends Num, Outputs extends Num> {

    //The system. Will be passed in the constructor
    private final LinearSystem<States, Inputs, Outputs> plant;

    //The LQR will compute the feedback gains for the system to move to the next state
    private final LinearQuadraticRegulator<States, Inputs, Outputs> controller;

    //The Kalman Filter fuses sensor/encoder values and resultant state to get the true state
    private final KalmanFilter<States, Inputs, Outputs> observer;

    //The entire state space control loop.
    private final LinearSystemLoop<States, Inputs, Outputs> stateSpaceLoop;

    private Matrix<States, N1> setpoint = null;
    private boolean atSetpoint = false;
    private Matrix<States, N1> tolerance;

    //Copy of states and inputs needed only for Kalman Filter. Stupid compile / run time errors force us to do this.
    //So now you have to instantiate states and outputs in the generics AND the constructor, even though they always are equal
    private final Nat<States> statesNat;
    private final Nat<Outputs> outputsNat;

    //How long the looping period will be.
    //For commands, this will be 20 ms, passed as 0.02
    private final double dtSeconds;

    //Just instantiated here to avoid creating a new matrix every 20 ms.
    private Matrix<States, N1> error;

    // Sensor Fusion Fields
    private boolean useDualSensors = false;
    private double secondarySensorWeight = 0.0;  // 0.0 = trust primary only, 1.0 = equal weight
    private double lastPrimaryMeasurement = 0.0;
    private double lastSecondaryMeasurement = 0.0;
    private Matrix<States, States> kalmanFilterCovariance = null;

    /**
     * Creates a generic state space controller.
     *
     * @param statesNat      Runtime Nat for state count  e.g. N2.instance
     * 
     * @param outputsNat     Runtime Nat for output count e.g. N1.instance
     * 
     * @param plant          LinearSystem model from LinearSystemId factory
     * 
     * @param qCost          State cost vector (States x 1)
     *                       Higher values = more aggressive correction on that state
     * 
     * @param rCost          Input cost vector (Inputs x 1)
     *                       Higher values = more conservative voltage usage
     * 
     * @param modelStdDevs   Model uncertainty per state (States x 1)
     *                       Higher = trust encoder more than model
     * 
     * @param encoderStdDevs Measurement noise per output (Outputs x 1)
     *                       Higher = trust model more than encoder
     * 
     * @param maxVoltage     Maximum voltage the controller can output. 
     *                       Usually we will put this as 12, but in case of voltage pdp issus we can limit max voltage
     * 
     * @param tolerance      Per-state tolerance for atSetpoint() (States x 1)
     * 
     * @param dtSeconds Control loop period, always 0.02 for commands
     */

    public SSController(
            Nat<States> statesNat,   //Just states value. Only used for Kalman Filter
            Nat<Outputs> outputsNat, //Just outputs value. Only used for Kalman Filter
            LinearSystem<States, Inputs, Outputs> plant,
            Vector<States> qCost,
            Vector<Inputs> rCost,
            Matrix<States, N1> modelStdDevs,
            Matrix<Outputs, N1> encoderStdDevs,
            double maxVoltage,
            Matrix<States, N1> tolerance,
            double dtSeconds) {

        this.statesNat     = statesNat;
        this.outputsNat    = outputsNat;
        this.plant         = plant;
        this.tolerance     = tolerance;
        this.dtSeconds = dtSeconds;

        // LQR computes optimal feedback gains
        controller = new LinearQuadraticRegulator<>(
            plant,
            qCost,
            rCost,
            dtSeconds
        );

        //Kalman Filter corrects predicted values
        observer = new KalmanFilter<>(
            statesNat,
            outputsNat,
            plant,
            modelStdDevs,
            encoderStdDevs,
            dtSeconds
        );

        stateSpaceLoop = new LinearSystemLoop<>(
            plant,
            controller,
            observer,
            maxVoltage,
            dtSeconds
        );

        stateSpaceLoop.reset(new Matrix<>(statesNat, Nat.N1()));
    }

    //Wraps all variables not relating to the system inside SSControllerConfigs
    public SSController(
            LinearSystem<States, Inputs, Outputs> plant,
            SSControllerConfigs<States, Inputs, Outputs> configs) {

        this.statesNat     = configs.getStatesNat();
        this.outputsNat    = configs.getOutputsNat();
        this.plant         = plant;
        this.tolerance     = configs.getTolerance();
        this.dtSeconds = configs.getdtSeconds();

        // LQR computes optimal feedback gains
        controller = new LinearQuadraticRegulator<>(
            plant,
            configs.getQCost(),
            configs.getRCost(),
            dtSeconds
        );

        //Kalman Filter corrects predicted values
        observer = new KalmanFilter<>(
            statesNat,
            outputsNat,
            plant,
            configs.getModelStdDevs(),
            configs.getEncoderStdDevs(),
            dtSeconds
        );

        stateSpaceLoop = new LinearSystemLoop<>(
            plant,
            controller,
            observer,
            configs.getMaxVoltage(),
            dtSeconds
        );

        stateSpaceLoop.reset(new Matrix<>(statesNat, Nat.N1()));
    }

    

    // -----------------------------------------------------------------------
    // Control Loop
    // -----------------------------------------------------------------------

    /**
     * Set the desired state vector.
     * @param endState desired state as column vector
     */
    public void setSetpoint(Matrix<States, N1> endState) {
        setpoint = endState;
        stateSpaceLoop.setNextR(endState);
    }

    /**
     * Run one iteration of the control loop.
     * Call every periodic loop and apply the returned voltage to your motor.
     * @param currentOutputs what your sensors measure. This is NOT the full state vector, only what is measured
     * @return voltage to apply to the motor
     */
    public double calculate(Matrix<Outputs, N1> currentOutputs) {
        if (setpoint == null) {
            DriverStation.reportError("[SSController] setpoint not set.", false);
            return 0.0;
        }

        // Fuse dual sensors if enabled
        Matrix<Outputs, N1> fusedOutputs = currentOutputs;
        if (useDualSensors && currentOutputs.getNumRows() >= 1) {
            fusedOutputs = fuseSensorReadings(currentOutputs);
        }

        stateSpaceLoop.correct(fusedOutputs);
        stateSpaceLoop.predict(dtSeconds);

        // Update Kalman filter covariance estimate
        kalmanFilterCovariance = stateSpaceLoop.getObserver().getP();

        error = stateSpaceLoop.getXHat().minus(setpoint);
        atSetpoint = isAtSetpoint(error);

        return stateSpaceLoop.getU(0);
    }

    /**
     * Fuses primary sensor with secondary sensor using weighted average.
     * @param primarySensorOutput primary sensor measurement
     * @return fused sensor reading
     */
    private Matrix<Outputs, N1> fuseSensorReadings(Matrix<Outputs, N1> primarySensorOutput) {
        lastPrimaryMeasurement = primarySensorOutput.get(0, 0);
        
        // Create fused output: weighted combination of primary and secondary
        double fusedValue = (lastPrimaryMeasurement * (1.0 - secondarySensorWeight)) + 
                            (lastSecondaryMeasurement * secondarySensorWeight);
        
        return MatBuilder.fill(outputsNat, Nat.N1(), fusedValue);
    }

    /**
     * Reset controller and observer to a known state.
     * Call on disable or when mechanism is at a known position.
     *
     * @param currentState current measured state as column vector
     */
    public void reset(Matrix<States, N1> currentState) {
        stateSpaceLoop.reset(currentState);
        setpoint = currentState;
        atSetpoint = false;
    }

    // -----------------------------------------------------------------------
    // Setters and Getters
    // -----------------------------------------------------------------------

    /** Returns true if the estimated state is within tolerance of the setpoint. 
     * Done by checking error (setpoint state - current state) against the given tolerance for each value.
    */
    private boolean isAtSetpoint(Matrix<States, N1> error) {
        for (int i = 0; i < error.getNumRows(); i++) {
            if (Math.abs(error.get(i, 0)) > tolerance.get(i, 0)) {
                return false;
            }
        }
        return true;
    }

    /** Getter for atSetpoint */
    public boolean atSetpoint() {
        return atSetpoint;
    }

    /** Returns the current setpoint state vector. */
    public Matrix<States, N1> getSetpoint() {
        return setpoint;
    }

    /** Returns the Kalman filter's current state estimate. */
    public Matrix<States, N1> getEstCurrentState() {
        return stateSpaceLoop.getXHat();
    }

    /** Returns the most recently computed voltage output. */
    public double getVoltage() {
        return stateSpaceLoop.getU(0);
    }

    /**
     * Update per-state tolerance for atSetpoint().
     * @param tolerance States x 1 matrix of per-state tolerances
     */
    public void setTolerance(Matrix<States, N1> tolerance) {
        this.tolerance = tolerance;
    }

    // -----------------------------------------------------------------------
    // Sensor Fusion Methods
    // -----------------------------------------------------------------------

    /**
     * Enable dual sensor fusion for state estimation.
     * Fuses primary sensor with secondary sensor (e.g., pose estimation).
     * @param weight weighting for secondary sensor (0.0 = primary only, 1.0 = equal weight)
     */
    public void enableDualSensorFusion(double weight) {
        this.useDualSensors = true;
        this.secondarySensorWeight = Math.max(0.0, Math.min(1.0, weight));
    }

    /** Disable dual sensor fusion - uses only primary sensor. */
    public void disableDualSensorFusion() {
        this.useDualSensors = false;
    }

    /**
     * Update the secondary sensor measurement.
     * Call this before calculate() with the latest measurement.
     * @param secondarySensorReading the secondary measurement (e.g., pose estimation)
     */
    public void setSecondarySensorReading(double secondarySensorReading) {
        this.lastSecondaryMeasurement = secondarySensorReading;
    }

    /**
     * Get the estimated uncertainty of the Kalman filter's state estimate.
     * Higher values = more uncertain. Based on the trace of the error covariance matrix.
     * @return uncertainty metric (sum of diagonal covariance elements)
     */
    public double getEstimateUncertainty() {
        if (kalmanFilterCovariance == null) {
            return 9999.0;  // No estimate yet
        }
        
        // Uncertainty is the sum of diagonal elements (trace) of covariance matrix
        double uncertainty = 0.0;

        for (int i = 0; i < kalmanFilterCovariance.getNumRows(); i++) {
            uncertainty += kalmanFilterCovariance.get(i, i);
        }
        return uncertainty;
    }

    /**
     * Get the confidence level as a percentage (inverse of uncertainty).
     * Higher = more confident in state estimate.
     * @return confidence as a value between 0.0 and 1.0 (typically 0.95+)
     */
    public double getEstimateConfidence() {
        double uncertainty = getEstimateUncertainty();
        if (uncertainty == 0.0) return 1.0;
        
        // Simple inverse confidence metric
        return Math.min(1.0, 1.0 / (1.0 + uncertainty));
    }

    /**
     * Check if the Kalman filter estimate is currently reliable.
     * Useful to decide whether to trust the estimate or fall back to raw sensor.
     * @param confidenceThreshold minimum acceptable confidence (0.0 to 1.0)
     * @return true if confidence > threshold
     */
    public boolean isEstimateReliable(double confidenceThreshold) {
        return getEstimateConfidence() > confidenceThreshold;
    }

    /** Get the last primary sensor measurement that was fused. */
    public double getLastPrimarySensorReading() {
        return lastPrimaryMeasurement;
    }

    /** Get the last secondary sensor measurement that was fused. */
    public double getLastSecondarySensorReading() {
        return lastSecondaryMeasurement;
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
