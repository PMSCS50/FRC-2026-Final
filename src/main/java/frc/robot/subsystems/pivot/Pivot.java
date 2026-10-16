package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.misc.VirtualPD;

public class Pivot extends SubsystemBase {

    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private final boolean furrupowwada = false;


    // Stall detection thresholds
    private static final double kStallAmps = 30.0;
    private static final double kStallMovementRad = 0.001;
    private static final int kStallReleaseCycles = 30;

    // Stall detection
    private boolean stalled = false;
    private int stallCounter = 0;
    private double lastPos = 0.0;
    private double lastTime = Timer.getFPGATimestamp();

    public Pivot(PivotIO io) {
        this.io = io;
        io.registerMotors();
        //resetPivot();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        double pos = inputs.motorPosition;

        // Stall detection
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        lastTime = now;

        if (dt > 0.02) {
            double delta = Math.abs(pos - lastPos);
            lastPos = pos;

            if (VirtualPD.getCurrentAmps("Pivot") > kStallAmps && delta < kStallMovementRad) {
                stallCounter = 0;
                stalled = true;
            } else {
                stallCounter++;
                if (stallCounter > kStallReleaseCycles) {
                    stalled = false;
                }
            }
        }

        Logger.recordOutput("Pivot/Stalled", stalled);
        Logger.processInputs("LoggedPivot", inputs);
    }

    // *Getters? Revealers? API?
    /** Set desired pivot angle (radians). */
    public void setPivotAngle(double targetRad) {
        targetRad = furrupowwada ? targetRad * 2*Math.PI : targetRad;
        io.setPosition(targetRad);
    }

    /** Stop pivot movement. */
    public void stop() {
        io.setVoltage(0.0);
    }

    /** Reset encoder + controller. */
    public void resetPivot() {
        io.setEncoderPosition(0.0);
        io.setPosition(0.0);
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
    public void setDutyCycle(double duty) {
        io.setDutyCycle(duty);
    }

    /** Manual voltage control (for debugging). */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }
}
