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
 * Generic state space state controller. ONLY HANDLES state
 * Given kV and kA from SysID, handles the full LQR + Kalman + feedforward loop
 * for any single-axis state-controlled mechanism (flywheel, shooter, intake, etc.)
 *
 * Usage:
 *   1. Instantiate with kV, kA, and tuning parameters
 *   2. Call setSetPoint() to set desired state
 *   3. Call calculate() every loop with current measured state
 *   4. Apply the returned voltage to your motor
 */
public class SSController<States extends Num,Inputs extends Num,Outputs extends Num> {

    private final LinearSystem<States, Inputs, Outputs> plant;

    private final LinearQuadraticRegulator<States, Inputs, Outputs> controller;

    private final KalmanFilter<States, Inputs, Outputs> observer;

    private final LinearSystemLoop<States, Inputs, Outputs> stateSpaceLoop;

    private double setpointState = 0.0;
    private boolean atGoal = false;
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
     * @param tolerance             state tolerance for atGoal() check ec.
     * @param loopPeriodSecs        Control loop period (usually 0.020).
     */
    public SSController(
            LinearSystem<States, Inputs, Outputs> plant,
            double qstate,
            double rVoltage,
            double modelStdDev,
            double encoderStdDev,
            double tolerance,
            double loopPeriodSecs) {

        this.tolerance = tolerance;

        //System plant is given in constructor.
        this.plant = plant;

        // LQR — computes optimal feedback gains
        // Q penalizes state error, R penalizes control effort
        controller = new LinearQuadraticRegulator<>(
            plant,
            VecBuilder.fill(qstate),   // state cost
            VecBuilder.fill(rVoltage),     // control effort cost
            loopPeriodSecs
        );

        // Kalman filter — fuses model prediction with encoder measurement
        observer = new KalmanFilter<>(
            States,
            Outputs,
            plant,
            VecBuilder.fill(modelStdDev),    // model uncertainty
            VecBuilder.fill(encoderStdDev),  // measurement uncertainty
            loopPeriodSecs
        );

        // Combines controller + observer + feedforward into one loop
        stateSpaceLoop = new LinearSystemLoop<>(
            plant,
            controller,
            observer,
            12.0,  // max voltage
            loopPeriodSecs
        );

        loop.reset(VecBuilder.fill(0.0));
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
        stateSpaceLoop.setNextR(VecBuilder.fill(endState));
    }

    /**
     * Run one iteration of the control loop.
     * Call this every periodic loop and apply the returned voltage to your motor.
     *
     * @param currentState current state from encoder 
     * @return voltage to apply to the motor
     */
    public double calculate(Matrix<States, N1> currentState) {
        // Update Kalman filter with current measurement
        stateSpaceLoop.correct(VecBuilder.fill(currentState));

        // Predict next state using model
        stateSpaceLoop.predict(0.020);

        // Check if at setpoint
        atGoal = Math.abs(currentState - setpointState) < tolerance;

        // Return optimal voltage
        return stateSpaceLoop.getU(0);
    }

    /**
     * Reset the controller and observer to a known state.
     * Call this when the mechanism starts from rest or re-enables.
     *
     * @param currentState current measured state
     */
    public void reset(Matrix<States, N1> currentState) {
        stateSpaceLoop.reset(VecBuilder.fill(currentState));
        setpointState = currentState;
        atGoal = false;
    }

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    /** Returns true if the mechanism is within tolerance of the setpoint state. */
    public boolean atSetpoint() {
        return atGoal;
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

    /** Update the state tolerance for atGoal(). */
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }
}