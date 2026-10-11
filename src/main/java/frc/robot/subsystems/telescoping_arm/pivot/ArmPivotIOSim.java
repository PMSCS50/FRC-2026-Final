package frc.robot.subsystems.telescoping_arm.pivot;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.misc.VirtualPD;
import frc.robot.Constants.ArmConstants;

public class ArmPivotIOSim implements ArmPivotIO {

    private final PIDController pivotController;
    private boolean motionMagic = false;
    private double setpoint = 0.0;
    

    private final SingleJointedArmSim pivotSim;

    public ArmPivotIOSim() {
        pivotController = new PIDController(3.0, 0.0, 0.02);

        pivotSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(3),
            ArmConstants.pivotGearing, // gear ratio
            1, // moment of inertia
            Units.inchesToMeters(30), // arm length
            0, // smallest angle
            110, // largest angle
            true,   // simulate gravity
            0     // starting angle
        );
    }

    @Override
    public void registerMotors() {
        VirtualPD.registerMotor(() -> Amps.of(pivotSim.getCurrentDrawAmps()), "Pivot (Arm)");
    }

    @Override
    public void updateInputs(ArmPivotIOInputs inputs) {
        pivotSim.update(0.02);

        inputs.position = pivotSim.getAngleRads();
        inputs.velocity = pivotSim.getVelocityRadPerSec();

        if (motionMagic) {
            double volts = MathUtil.clamp(pivotController.calculate(inputs.position, setpoint), -12.0, 12.0);
            pivotSim.setInputVoltage(volts);
        }
    }

    private void setVoltage(double volts) {
        motionMagic = false;
        pivotSim.setInputVoltage(volts);
    }

    @Override
    public void setDutyCycle(double duty) {
        setVoltage(duty * 12.0);
    }

    @Override
    public void setMotionMagicPosition(double position) {
        motionMagic = true;
        setpoint = position;
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {}

    
}
