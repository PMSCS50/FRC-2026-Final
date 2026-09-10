package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

// variables: target rpm, current rpm, current, voltage

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIO.IntakeIOInputs inputs = new IntakeIOInputs();

    private final PIDController pid = new PIDController(.01, 0, 0);
    private double targetRPM = 0;

    // constructor
    public Intake(IntakeIO io) {
        this.io = io;
        io.registerMotors();
    }

    // periodic method to update inputs and log outputs
    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    // actual complex commands for the motor (not just instantiation or voltage control) or commands that dont need to be differentiated between sim and real
    public void stop() {
        io.setVoltage(0);
    }

    public void runOpenLoop(double percent) {
        io.setVoltage(percent * 12);
    }

    public void runPID(double percent) {
        targetRPM = percent * 5676;
        double voltage = pid.calculate(inputs.motorVelocity, targetRPM);
        io.setVoltage(voltage);
    }

    // getters
    public double getRPM() {
        return inputs.motorVelocity;
    }

}