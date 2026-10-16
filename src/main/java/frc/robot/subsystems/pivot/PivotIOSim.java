package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.robot.util.misc.VirtualPD;

public class PivotIOSim implements PivotIO {

    private final PIDController pivotPID;
    private final ArmFeedforward pivotFF;
    private boolean pidControl = false;
    private double setpoint = 0.0;

    private final SingleJointedArmSim armSim;
    public final LoggedMechanism2d pivotVisual = new LoggedMechanism2d(3, 3);
    private final LoggedMechanismRoot2d pivotRoot = pivotVisual.getRoot("Pivot", 1.5, 1);
    private final LoggedMechanismLigament2d pivotLigament = new LoggedMechanismLigament2d("PivotArm", 1.0, 0.0);

    public PivotIOSim() {
        pivotRoot.append(pivotLigament);
        this.armSim = new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            50, // gear ratio
            0.05, // moment of inertia
            0.14, // arm length
            0, // smallest angle
            3*Math.PI/4, // largest angle
            true,   // simulate gravity
            Math.PI / 3     // starting angle
        );

        this.pivotPID = new PIDController(5, 0, 0.5);
        this.pivotFF = new ArmFeedforward(0, 1.3, 0);
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
        setpoint = rotations / 50.0 * 2.0 * Math.PI;
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
            double pidVolts = pivotPID.calculate(inputs.motorPosition, setpoint);
            double ffVolts = pivotFF.calculate(inputs.motorPosition, 0);
            appliedVolts = MathUtil.clamp(pidVolts + ffVolts, -12.0, 12.0);
            inputs.motorVoltage = appliedVolts;
            armSim.setInputVoltage(appliedVolts);

            Logger.recordOutput("Pivot/Sim/PIDvolts", pidVolts);
            Logger.recordOutput("Pivot/Sim/FFvolts", ffVolts);
            Logger.recordOutput("Pivot/Sim/TotalVolts", appliedVolts);
        }

        armSim.update(0.02);

        inputs.motorPosition = armSim.getAngleRads();
        inputs.motorVelocity = armSim.getVelocityRadPerSec();
        pivotLigament.setAngle(Math.toDegrees(armSim.getAngleRads()));

        Logger.recordOutput("Pivot/Sim/Position", inputs.motorPosition);
        Logger.recordOutput("Pivot/Sim/Setpoint", setpoint);
        Logger.recordOutput("Pivot/Visual", pivotVisual);

    }
}