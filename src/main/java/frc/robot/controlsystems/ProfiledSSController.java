package frc.robot.controlsystems;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


/**
 * Specific SSController that adds trapezoidal profiling to the setpoint.
 * However, this is solely a position-velocity profiled controller.
 * There are two fixed states: position and velocity. This is only for mechanisms where those are the two states we care about.
 * Should not be used as a replacement for SSController
 */
public class ProfiledSSController<Inputs extends Num, Outputs extends Num> {

    //The system. Will be passed in the constructor
    private final LinearSystem<N2, Inputs, Outputs> plant;

    //The LQR will compute the feedback gains for the system to move to the next state
    private final LinearQuadraticRegulator<N2, Inputs, Outputs> controller;

    //The Kalman Filter fuses sensor/encoder values and resultant state to get the true state
    private final KalmanFilter<N2, Inputs, Outputs> observer;

    //The entire state space control loop.
    private final LinearSystemLoop<N2, Inputs, Outputs> stateSpaceLoop;

    private final TrapezoidProfile profile;
    
    private final TrapezoidProfile.Constraints constraints;

    private final TrapezoidProfile.State trapezoidState;


    private Matrix<N2, N1> setpoint;
    private boolean atSetpoint = false;
    private Matrix<N2, N1> tolerance;

    //Copy of states and inputs needed only for Kalman Filter. Stupid compile / run time errors force us to do this.
    //So now you have to instantiate states and outputs in the generics AND the constructor, even though they always are equal
    private final Nat<N2> statesNat;
    private final Nat<Outputs> outputsNat;

    //How long the looping period will be.
    //For commands, this will be 20 ms, passed as 0.02
    private final double dtSeconds;

    //Just instantiated here to avoid creating a new matrix every 20 ms.
    private Matrix<N2, N1> error;

    /**
     * Creates a generic state space controller.
     *
     * @param statesNat      Runtime Nat for state count  e.g. N2.instance
     * 
     * @param outputsNat     Runtime Nat for output count e.g. N1.instance
     * 
     * @param plant          LinearSystem model from LinearSystemId factory
     * 
     * @param qCost          State cost vector (N2 x 1)
     *                       Higher values = more aggressive correction on that state
     * 
     * @param rCost          Input cost vector (Inputs x 1)
     *                       Higher values = more conservative voltage usage
     * 
     * @param modelStdDevs   Model uncertainty per state (N2 x 1)
     *                       Higher = trust encoder more than model
     * 
     * @param encoderStdDevs Measurement noise per output (Outputs x 1)
     *                       Higher = trust model more than encoder
     * 
     * @param maxVoltage     Maximum voltage the controller can output. 
     *                       Usually we will put this as 12, but in case of voltage pdp issus we can limit max voltage
     * 
     * @param tolerance      Per-state tolerance for atSetpoint() (N2 x 1)
     * 
     * @param dtSeconds Control loop period, always 0.02 for commands
     * 
     * @param constraints     Constraints for trapezoidal profile (max velocity and acceleration)
     */

    public SSController(
            Nat<N2> statesNat,   //Just states value. Only used for Kalman Filter
            Nat<Outputs> outputsNat, //Just outputs value. Only used for Kalman Filter
            LinearSystem<N2, Inputs, Outputs> plant,
            Vector<N2> qCost,
            Vector<Inputs> rCost,
            Matrix<N2, N1> modelStdDevs,
            Matrix<Outputs, N1> encoderStdDevs,
            double maxVoltage,
            Matrix<N2, N1> tolerance,
            double dtSeconds,
            TrapezoidProfile.Constraints constraints) {

        this.statesNat     = statesNat;
        this.outputsNat    = outputsNat;
        this.plant         = plant;
        this.tolerance     = tolerance;
        this.dtSeconds = dtSeconds;
        this.constraints = constraints;
        this.profile = new TrapezoidProfile(constraints);
        this.trapezoidState = new TrapezoidProfile.State();

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
            LinearSystem<N2, Inputs, Outputs> plant,
            SSControllerConfigs<N2, Inputs, Outputs> configs,
            TrapezoidProfile.Constraints constraints) {

        this.statesNat     = configs.getN2Nat();
        this.outputsNat    = configs.getOutputsNat();
        this.plant         = plant;
        this.tolerance     = configs.getTolerance();
        this.dtSeconds = configs.getdtSeconds();
        this.constraints = constraints;
        this.profile = new TrapezoidProfile(constraints);
        this.trapezoidState = new TrapezoidProfile.State();

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
    public void setSetpoint(double position, double velocity) {
        setpoint = VecBuilder.fill(position, velocity);
        stateSpaceLoop.setNextR(setpoint);
    }

    /**
     * Run one iteration of the control loop.
     * Call every periodic loop and apply the returned voltage to your motor.
     * @param currentOutputs what your sensors measure. This is NOT the full state vector, only what is measured
     * @return voltage to apply to the motor
     */
    public double calculate(Matrix<Outputs, N1> currentOutputs) {

        trapezoidState = profile.calculate(dtSeconds, trapezoidState, setpoint);
        stateSpaceloop.setNextR(trapezoidState.position, trapezoidState.velocity);

        stateSpaceLoop.correct(currentOutputs);
        stateSpaceLoop.predict(dtSeconds);

        error = stateSpaceLoop.getXHat().minus(setpoint);
        atSetpoint = isAtSetpoint(error);

        return stateSpaceLoop.getU(0);
    }

    /**
     * Reset controller and observer to a known state.
     * Call on disable or when mechanism is at a known position.
     *
     * @param currentState current measured state as column vector
     */
    public void reset(Matrix<N2, N1> currentState) {
        stateSpaceLoop.reset(currentState);
        setpoint = currentState;
        trapezoidState = new TrapezoidProfile.State(currentState.get(0, 0), currentState.get(1, 0));
        atSetpoint = false;
    }

    // -----------------------------------------------------------------------
    // Setters and Getters
    // -----------------------------------------------------------------------

    /** Getter for atSetpoint */
    public boolean isAtSetpoint() {
        if (!profile.isFinished(dtSeconds)) {
            return false;
        }
        
        error = setpoint.minus(stateSpaceLoop.getXHat());
        
        return isAtSetpointInternal(error);
    }

    private boolean isAtSetpointInternal(Matrix<N2, N1> error) {
        for (int i = 0; i < error.getNumRows(); i++) {
            if (Math.abs(error.get(i, 0)) > tolerance.get(i, 0)) {
                return false;
            }
        }
        return true;
    }

    /** Returns the current setpoint state vector. */
    public Matrix<N2, N1> getSetpoint() {
        return setpoint;
    }

    /** Returns the Kalman filter's current state estimate. */
    public Matrix<N2, N1> getEstCurrentState() {
        return stateSpaceLoop.getXHat();
    }

    /** Returns the most recently computed voltage output. */
    public double getVoltage() {
        return stateSpaceLoop.getU(0);
    }

    /**
     * Update per-state tolerance for atSetpoint().
     * @param tolerance N2 x 1 matrix of per-state tolerances
     */
    public void setTolerance(Matrix<N2, N1> tolerance) {
        this.tolerance = tolerance;
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
