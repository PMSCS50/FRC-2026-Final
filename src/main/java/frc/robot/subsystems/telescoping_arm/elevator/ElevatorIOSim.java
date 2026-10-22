package frc.robot.subsystems.telescoping_arm.elevator;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.misc.VirtualPD;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ElevatorIOSim implements ElevatorIO {

    private boolean motionMagic = false;
    private double setpoint = 0.0;
    private double volts = 0;

    private PIDController elevatorController;
    private final ElevatorSim sim;

    public ElevatorIOSim() {
        elevatorController = new PIDController(3.0, 0.0, 0.02);

        sim = new ElevatorSim(
            0.115, // |pulled out of Suhas' ass
            0.02, // |pulled out of Suhas' ass
            DCMotor.getKrakenX60(3), // 3 motors
            1, // min height
            5, // max height
            true, // Simulate gravity
            Units.inchesToMeters(31.032)
        );

        registerMotors();
    }

    private void registerMotors() {
        VirtualPD.registerMotor(() -> Amps.of(sim.getCurrentDrawAmps()), "Elevator");
    }

    private void setVoltage(double volts) {
        motionMagic = false;
        sim.setInputVoltage(volts);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        volts = dutyCycle * RobotController.getBatteryVoltage();
        setVoltage(volts);
    }

    @Override
    public void setMotionMagicPosition(double positionMeters) {
        motionMagic = true;
        setpoint = positionMeters;
    }

    @Override
    public void stop() {
        setVoltage(0);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        return; // Sim does not care about neutral mode
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sim.update(0.02);

        inputs.position = Degrees.of(sim.getPositionMeters() / ArmConstants.ELEVATOR_POSITION_COEFFICIENT);
        inputs.velocity = RadiansPerSecond.of(sim.getVelocityMetersPerSecond());
        inputs.acceleration = RadiansPerSecondPerSecond.of(0); // WPILib sim does not expose accel
        inputs.appliedVoltage = Volts.of(volts);
        inputs.atSetpoint = false; // Sim does not know MM target
        inputs.extension = sim.getPositionMeters();

        if (motionMagic) {
            double output = elevatorController.calculate(inputs.extension, setpoint);
            sim.setInputVoltage(output);
        }

        Logger.recordOutput("Elevator/Position", inputs.position);
    }
}
