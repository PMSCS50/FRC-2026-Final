package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// variables: target rpm, current rpm, current, voltage

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // constructor
    public Intake(IntakeIO io) {
        this.io = io;
        io.registerMotors();
    }

    // periodic method to update inputs and log outputs
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LoggedIntake", inputs);
    }

    // actual complex commands for the motor (not just instantiation or voltage control) or commands that dont need to be differentiated between sim and real
    public void stop() {
        io.setVoltage(0);
    }

    public void runOpenLoop(double percent) {
        io.setVoltage(percent * 12);
    }

    public void runPID(double percent) {
        io.runPID(percent);
    }

    // getters
    public double getRPM() {
        return inputs.motorVelocity;
    }

}