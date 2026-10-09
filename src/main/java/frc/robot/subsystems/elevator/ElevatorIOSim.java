package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;

public class ElevatorIOSim implements ElevatorIO {

    private final ElevatorSim sim =
        new ElevatorSim(
            0.115, // |pulled out of Suhas' ass
            0, // |pulled out of Suhas' ass
            DCMotor.getKrakenX60(3), // 3 motors
            0, // min height
            10, // max height
            true, // Simulate gravity
            0
        );


    private double volts = 0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sim.update(0.02);

        inputs.position = Units.Degrees.of(sim.getPositionMeters() / ElevatorConstants.ELEVATOR_POSITION_COEFFICIENT);
        inputs.velocity = Units.RadiansPerSecond.of(sim.getVelocityMetersPerSecond());
        inputs.acceleration = Units.RadiansPerSecondPerSecond.of(0); // WPILib sim does not expose accel
        inputs.appliedVoltage = Units.Volts.of(volts);
        inputs.atSetpoint = false; // Sim does not know MM target
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        volts = dutyCycle * RobotController.getBatteryVoltage();
        sim.setInputVoltage(volts);
    }

    @Override
    public void setMotionMagicPosition(double positionMeters) {
        // Simple P controller for sim
        double error = positionMeters - sim.getPositionMeters();
        double volts = error * 3; // |pulled out of Suhas' ass
        sim.setInputVoltage(volts);
    }

    @Override
    public void stop() {
        sim.setInputVoltage(0);
    }

    @Override
    public void setNeutralMode(Object neutralMode) {
        // ignored in sim
    }
}
