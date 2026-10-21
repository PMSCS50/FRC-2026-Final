package frc.robot.subsystems.telescoping_arm.pivot;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.misc.VariableArmSim;
import frc.robot.util.misc.VirtualPD;
import frc.robot.Constants.ArmConstants;

public class ArmPivotIOSim implements ArmPivotIO {

    private final PIDController pivotController;
    private boolean motionMagic = false;
    private double setpoint = 0.0;

    private final DoubleSupplier elevatorPositionMeters;

    private final VariableArmSim pivotSim;
    public final LoggedMechanism2d armPivotVisual = new LoggedMechanism2d(3, 3);
    private final LoggedMechanismRoot2d armPivotRoot = armPivotVisual.getRoot("Pivot", 1.5, 0.5);
    private final LoggedMechanismLigament2d armPivotLigament = new LoggedMechanismLigament2d("PivotArm", 1.0, 90);

    public ArmPivotIOSim(DoubleSupplier elevatorPositionMeters) {
        this.elevatorPositionMeters = elevatorPositionMeters;
        pivotController = new PIDController(3.0, 0.0, 0.02);

        pivotSim = new VariableArmSim(
            13.6,
            ArmConstants.armMasses,
            ArmConstants.armLengths,
            DCMotor.getKrakenX60(3),
            ArmConstants.pivotGearing, // gear ratio
            VariableArmSim.calculateMOI(elevatorPositionMeters.getAsDouble(), ArmConstants.armMasses, ArmConstants.armLengths), // moment of inertia
            elevatorPositionMeters, // arm length
            0, // smallest angle
            110
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
