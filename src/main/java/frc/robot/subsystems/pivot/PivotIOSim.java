package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.misc.VirtualPD;

public class PivotIOSim implements PivotIO {

    private final PIDController pivotController;
    private boolean pidControl = false;
    private double setpoint = 0.0;

    private final SingleJointedArmSim armSim;

    public PivotIOSim() {
        this.armSim = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            3, // gear ratio
            2.0, // moment of inertia
            0.302441737, // arm length
            0, // smallest angle
            3*Math.PI/4, // largest angle
            false,   // simulate gravity
            Math.PI / 3     // starting angle
        );

        this.pivotController = new PIDController(1.5, 0.0, 3.5);
    }

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
        inputs.motorVoltage = appliedVolts;
        inputs.motorPosition = armSim.getAngleRads();
        inputs.motorVelocity = armSim.getVelocityRadPerSec();

        if (pidControl) {
            appliedVolts = MathUtil.clamp(pivotController.calculate(inputs.motorPosition, setpoint), -12.0, 12.0);
            inputs.motorVoltage = appliedVolts;
            armSim.setInputVoltage(appliedVolts);

            inputs.motorPosition = armSim.getAngleRads();
            inputs.motorVelocity = armSim.getVelocityRadPerSec();
        }
        armSim.update(0.02);
    }
}