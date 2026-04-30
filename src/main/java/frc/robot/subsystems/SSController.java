package frc.robot.util;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Matrix;
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

    //Needed only for Kalman Filter. Stupid compile / run time errors force us to do this.
    private final Nat<States> statesNat;
    private final Nat<Outputs> outputsNat;

    //How long the looping period will be.
    //For commands, this will be 20 ms, passed as 0.02
    private final double loopPeriodSecs;

    //Just instantiated here to avoid creating a new matrix every 20 ms.
    private Matrix<States, N1> error;

    /**
     * Creates a generic state space controller.
     *
     * @param statesNat      Runtime Nat for state count  e.g. N2.instance
     * @param outputsNat     Runtime Nat for output count e.g. N1.instance
     * @param plant          LinearSystem model from LinearSystemId factory
     * @param qCost          State cost matrix (States x States diagonal)
     *                       Higher diagonal values = more aggressive correction on that state
     * @param rCost          Input cost matrix (Inputs x Inputs diagonal)
     *                       Higher values = more conservative voltage usage
     * @param modelStdDevs   Model uncertainty per state (States x 1)
     *                       Higher = trust encoder more than model
     * @param encoderStdDevs Measurement noise per output (Outputs x 1)
     *                       Higher = trust model more than encoder
     * @param tolerance      Per-state tolerance for atSetpoint() (States x 1)
     * @param loopPeriodSecs Control loop period, always 0.02 for commands
     */

    public SSController(
            Nat<States> statesNat,   //Just states value. Only used for Kalman Filter
            Nat<Outputs> outputsNat, //Just outputs value. Only used for Kalman Filter
            LinearSystem<States, Inputs, Outputs> plant,
            Matrix<States, States> qCost,
            Matrix<Inputs, Inputs> rCost,
            Matrix<States, N1> modelStdDevs,
            Matrix<Outputs, N1> encoderStdDevs,
            Matrix<States, N1> tolerance,
            double loopPeriodSecs) {

        this.statesNat     = statesNat;
        this.outputsNat    = outputsNat;
        this.plant         = plant;
        this.tolerance     = tolerance;
        this.loopPeriodSecs = loopPeriodSecs;

        // LQR computes optimal feedback gains
        controller = new LinearQuadraticRegulator<>(
            plant,
            qCost,
            rCost,
            loopPeriodSecs
        );

        //Kalman Filter corrects predicted values
        observer = new KalmanFilter<>(
            statesNat,
            outputsNat,
            plant,
            modelStdDevs,
            encoderStdDevs,
            loopPeriodSecs
        );

        stateSpaceLoop = new LinearSystemLoop<>(
            plant,
            controller,
            observer,
            12.0,
            loopPeriodSecs
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

        stateSpaceLoop.correct(currentOutputs);
        stateSpaceLoop.predict(loopPeriodSecs);

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

}