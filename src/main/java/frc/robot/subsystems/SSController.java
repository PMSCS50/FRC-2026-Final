package frc.robot.util;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Generic state space state controller.
 * Given the system, handles the full LQR + Kalman + feedforward loop
 *
 * Usage:
 *   1. Instantiate with a given system and tuning parameters
 *   2. Call setSetPoint() to set desired state
 *   3. Call calculate() every loop with current measured state
 *   4. Apply the returned voltage to your motor
 */
public class SSController<States extends Num,Inputs extends Num,Outputs extends Num> {

    private final LinearSystem<States, Inputs, Outputs> plant;

    private final LinearQuadraticRegulator<States, Inputs, Outputs> controller;

    private final KalmanFilter<States, Inputs, Outputs> observer;

    private final LinearSystemLoop<States, Inputs, Outputs> stateSpaceLoop;

    private Matrix<State, N1> setpointState = null;
    private boolean atSetpoint = false;
    private double tolerance;

    /**
     * Creates a state space state controller.
     *
     * @param qstate             State cost, or how much to penalize state error (rad/s).
     *                              Lower = more aggressive correction.
     * @param rVoltage              Control effort cost — how much to penalize voltage usage (V).
     *                              Lower = allows more aggressive voltage output.
     * @param modelStdDev           How much you trust the model vs measurement.
     *                              Higher = trust measurement more.
     * @param encoderStdDev         Encoder measurement noise standard deviation (rad/s).
     *                              Higher = trust model more.
     * @param tolerance             state tolerance for atSetpoint() check ec.
     */
    public SSController(
            LinearSystem<States, Inputs, Outputs> plant,
            Matrix<State, State> qstate,
            Matrix<State, State>  rVoltage,
            double modelStdDev,
            double encoderStdDev,
            double tolerance
        ) {

        this.tolerance = tolerance;

        //System plant is given in constructor.
        this.plant = plant;

        // LQR computes optimal feedback gains
        // Q penalizes state error, R penalizes control effort
        controller = new LinearQuadraticRegulator<>(
            plant,
            qstate,   // state cost
            rVoltage, // control effort cost
            0.020     //every 20 ms
        );

        // Kalman filter fuses model prediction with encoder measurement
        observer = new KalmanFilter<>(
            nat.get(plant.getC.getNumCols), // States.
            nat.get(plant.getC.getNumRows), // Outputs.
            plant,
            VecBuilder.fill(modelStdDev),    // model uncertainty
            VecBuilder.fill(encoderStdDev),  // measurement uncertainty
            0.020 //every 20 ms
        );

        // Combines controller + observer + feedforward into one loop
        stateSpaceLoop = new LinearSystemLoop<>(
            plant,
            controller,
            observer,
            12.0,  // max voltage
        );

        stateSpaceLoop.reset(VecBuilder.fill(0.0));
    }

    // -----------------------------------------------------------------------
    // Control Loop
    // -----------------------------------------------------------------------

    /**
     * Set the desired state.
     * Call this whenever the setpoint changes.
     *
     * @param endState desired state
     */
    public void setSetPoint(Matrix<States, N1> endState) {
        setpoint = endState;
        stateSpaceLoop.setNextR(endState);
    }

    /**
     * Calculates the next required voltage value for the system.
     * Call this every periodic loop and apply the returned voltage to your motor.
     *
     * @param currentState current state from encoder 
     * @return voltage to apply to the motor
     */
    public double calculate(Matrix<States, N1> currentState) {
        if (setpointState == null) {
            DriverStation.reportError("[SSController] setpoint is not defined.");
            return 0.0;
        }

        // Update Kalman filter with current measurement
        stateSpaceLoop.correct(currentState);

        // Predict next state using model
        stateSpaceLoop.predict(0.020);

        // Check if at setpoint
        atSetpoint = currentState.isEqual(setPointState, tolerance);

        // Return optimal voltage
        return stateSpaceLoop.getU();
    }

    /**
     * Reset the controller and observer to a known state.
     * Call this when the mechanism starts from rest or re-enables.
     *
     * @param currentState current measured state
     */
    public void reset(Matrix<States, N1> currentState) {
        stateSpaceLoop.reset(currentState);
        setpointState = currentState;
        atSetpoint = false;
    }

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    /** Returns true if the mechanism is within tolerance of the setpoint state. */
    public boolean atSetpoint() {
        return atSetpoint;
    }

    /** Returns the current setpoint state */
    public Matrix<States, N1> getSetpoint() {
        return setpointState;
    }

    /** Returns the Kalman filter's current state estimate */
    public Matrix<States, N1> getEstimatedState() {
        return stateSpaceLoop.getXHat();
    }

    /** Returns the most recently computed voltage output. */
    public double getVoltage() {
        return stateSpaceLoop.getU(0);
    }

    /** Update the state tolerance for atSetpoint(). */
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    private Map<Integer, Nat<?>> nat = Map.of(
        1,   Nat.N1(),
        2,   Nat.N2(),
        3,   Nat.N3(),
        4,   Nat.N4(),
        5,   Nat.N5(),
        6,   Nat.N6(),
        7,   Nat.N7(),
        8,   Nat.N8(),
        9,   Nat.N9(),
        10, Nat.N10(),
        11, Nat.N11(),
        12, Nat.N12(),
        13, Nat.N13(),
        14, Nat.N14(),
        15, Nat.N15(),
        16, Nat.N16(),
        17, Nat.N17(),
        18, Nat.N18(),
        19, Nat.N19(),
        20, Nat.N20(),
    );
}