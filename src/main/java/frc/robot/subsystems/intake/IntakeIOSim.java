package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.misc.VirtualPD;

public class IntakeIOSim implements IntakeIO {
    double kV = 12.0 / 5676.0;   // will need to be tuned later
    double kA = 0.001;           // will need to be tuned later

    private final LinearSystem<N1, N1, N1> system = LinearSystemId.identifyVelocitySystem(kV, kA);
    private final FlywheelSim sim = new FlywheelSim(system, DCMotor.getNEO(1), 1.0);
    private double appliedVolts = 0;

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
    }

    @Override
    public void registerMotors() {
        VirtualPD.registerMotor(() -> Amps.of(sim.getCurrentDrawAmps()), "Intake");
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        sim.setInputVoltage(appliedVolts);
        sim.update(.02);

        inputs.motorVoltage = appliedVolts;
        inputs.motorVelocity = sim.getAngularVelocityRadPerSec();
    }
}
