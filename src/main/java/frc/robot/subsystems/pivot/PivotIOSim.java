package frc.robot.subsystems.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.IntakeConstants;

public class PivotIOSim implements PivotIO {

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
        appliedVolts = volts;
        armSim.setInputVoltage(volts);
    }

    @Override
    public void setDutyCycle(double duty) {
        setVoltage(duty * 12.0);
    }

    @Override
    public void setPosition(double rotations) {
        double error = rotations - armSim.getAngleRads();
        setVoltage(error * 4.0);
    }

    @Override
    public void setEncoderPosition(double pos) {
        armSim.setState(pos, 0.0);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        armSim.update(0.02);

        inputs.motorAmperage = armSim.getCurrentDrawAmps();
        inputs.motorVoltage = appliedVolts;
        inputs.motorPosition = armSim.getAngleRads();
        inputs.motorVelocity = armSim.getVelocityRadPerSec();
    }
}