package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {

    private final PivotIO io;
    private final PivotIO.PivotIOInputs inputs = new PivotIO.PivotIOInputs();

    // Desired pivot angle (radians)
    private double goalRad = 0.0;

    // -----------------------------
    // Default tuning values
    // -----------------------------

    // PID gains (start simple)
    private static final double kP = 2.0;
    private static final double kI = 0.0;
    private static final double kD = 0.1;

    // Motion constraints
    private static final double kMaxVel = 4.0;      // rad/s
    private static final double kMaxAccel = 8.0;    // rad/s^2

    // Feedforward gains
    private static final double kS = 0.0;           // static friction
    private static final double kG = 0.5;           // gravity gain
    private static final double kV = 0.1;           // velocity gain
    private static final double kA = 0.0;           // acceleration gain

    // Stall detection thresholds
    private static final double kStallAmps = 30.0;
    private static final double kStallMovementRad = 0.001;
    private static final int kStallReleaseCycles = 30;

    // -----------------------------
    // Controllers
    // -----------------------------

    private final ProfiledPIDController pid =
        new ProfiledPIDController(
            kP, kI, kD,
            new TrapezoidProfile.Constraints(kMaxVel, kMaxAccel)
        );

    private final ArmFeedforward ff =
        new ArmFeedforward(kS, kG, kV, kA);

    // Stall detection
    private boolean stalled = false;
    private int stallCounter = 0;
    private double lastPos = 0.0;
    private double lastTime = Timer.getFPGATimestamp();

    public Pivot(PivotIO io) {
        this.io = io;
        pid.setGoal(goalRad);
        pid.reset(0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        double pos = inputs.motorPosition;
        double vel = inputs.motorVelocity;

        // PID + Feedforward → voltage
        double pidOut = pid.calculate(pos, goalRad);
        double ffOut = ff.calculate(goalRad, vel);
        double volts = pidOut + ffOut;

        io.setVoltage(volts);

        // Stall detection
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        lastTime = now;

        if (dt > 0.02) {
            double delta = Math.abs(pos - lastPos);
            lastPos = pos;

            if (inputs.motorAmperage > kStallAmps && delta < kStallMovementRad) {
                stallCounter = 0;
                stalled = true;
            } else {
                stallCounter++;
                if (stallCounter > kStallReleaseCycles) {
                    stalled = false;
                }
            }
        }
    }

    // *Getters? Revealers? API?
    /** Set desired pivot angle (radians). */
    public void setPivotAngle(double targetRad) {
        goalRad = targetRad;
        pid.setGoal(targetRad);
    }

    /** Stop pivot movement. */
    public void stop() {
        io.setVoltage(0.0);
    }

    /** Reset encoder + controller. */
    public void resetPivot() {
        io.setEncoderPosition(0.0);
        pid.reset(0.0);
        goalRad = 0.0;
    }

    /** Current pivot angle (radians). */
    public double getAngle() {
        return inputs.motorPosition;
    }

    /** Current pivot velocity (rad/s). */
    public double getVelocity() {
        return inputs.motorVelocity;
    }

    /** True if pivot is stalled. */
    public boolean isStalled() {
        return stalled;
    }

    /** True if pivot is at target within tolerance. */
    public boolean atAngle(double targetRad, double toleranceRad) {
        return Math.abs(getAngle() - targetRad) < toleranceRad;
    }

    /** Manual percent output (for commands or teleop override). */
    public void setManualDuty(double duty) {
        io.setDutyCycle(duty);
    }

    /** Manual voltage control (for debugging). */
    public void setManualVoltage(double volts) {
        io.setVoltage(volts);
    }
}
