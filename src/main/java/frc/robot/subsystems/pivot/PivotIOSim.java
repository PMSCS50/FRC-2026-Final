package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.misc.VirtualPD;

public class PivotIOSim implements PivotIO {

    private final PIDController pivotController = new PIDController(0.4, 0.0, 0.0);
    private boolean pidControl = false;
    private double setpoint = 0.0;

    private final SingleJointedArmSim armSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            1, // gear ratio
            1, // moment of inertia
            .5, // arm length
            0, // smallest angle
            Math.PI/2, // largest angle
            true,   // simulate gravity
            Math.PI/2     // starting angle
        );

    private double appliedVolts = 0.0;

    @Override
    public void setVoltage(double volts) {
        pidControl = false;
        appliedVolts = volts;
        armSim.setInputVoltage(volts);
    }

    @Override
    public void setDutyCycle(double duty) {
        setVoltage(duty * 12.0);
    }

    @Override
    public void setPosition(double rotations) {
        pidControl = true;
        setpoint = rotations;
    }

    @Override
    public void setEncoderPosition(double pos) {
        armSim.setState(pos, 0.0);
    }

    @Override
    public void registerMotors() {
        VirtualPD.registerMotor(() -> Amps.of(armSim.getCurrentDrawAmps()), "Pivot");
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        armSim.update(0.02);

        inputs.motorVoltage = appliedVolts;
        inputs.motorPosition = armSim.getAngleRads();
        inputs.motorVelocity = armSim.getVelocityRadPerSec();

        if (pidControl) {
            appliedVolts = pivotController.calculate(inputs.motorPosition, setpoint);
            inputs.motorVoltage = appliedVolts;
            armSim.setInputVoltage(appliedVolts);
        }
    }
}