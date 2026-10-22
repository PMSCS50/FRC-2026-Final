package frc.robot.subsystems.telescoping_arm.pivot;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
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

    private final PIDController armPivotController;
    private boolean motionMagic = false;
    private double setpoint = 0.0;
    private DoubleSupplier ff;

    private DoubleSupplier moi;
    private final double moiReference = VariableArmSim.calculateMOI(4, ArmConstants.armMasses, ArmConstants.armLengths);
    private final double kP = 0.05;
    private final double kI = 0.0;
    private final double kD = 0.015;

    private final VariableArmSim armPivotSim;
    public final LoggedMechanism2d armPivotVisual = new LoggedMechanism2d(10, 7);
    private final LoggedMechanismRoot2d armPivotRoot = armPivotVisual.getRoot("Pivot", 5, 2);
    private final LoggedMechanismLigament2d armPivotLigament = new LoggedMechanismLigament2d("PivotArm", 1.0, 90);

    public ArmPivotIOSim(DoubleSupplier elevatorPositionMeters) {
        moi = () -> VariableArmSim.calculateMOI(elevatorPositionMeters.getAsDouble(), ArmConstants.armMasses, ArmConstants.armLengths);
        armPivotRoot.append(armPivotLigament);

        armPivotSim = new VariableArmSim(
            ArmConstants.armMasses,
            ArmConstants.armLengths,
            DCMotor.getKrakenX60(3),
            ArmConstants.pivotGearing, // gear ratio
            moi.getAsDouble(), // moment of inertia
            elevatorPositionMeters, // arm length
            Math.toRadians(0), // smallest angle
            Math.toRadians(180), // largest angle
            true // simulate gravity
        );

        ff = () -> armPivotSim.calculateFF(Math.toRadians(setpoint));
        armPivotController = new PIDController(kP, kI, kD);
    }

    @Override
    public void registerMotors() {
        VirtualPD.registerMotor(() -> Amps.of(armPivotSim.getCurrentDrawAmps()), "Pivot (Arm)");
    }

    private void setVoltage(double volts) {
        motionMagic = false;
        armPivotSim.setInputVoltage(volts);
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

    @Override
    public void updateInputs(ArmPivotIOInputs inputs) {
        armPivotSim.update(0.02);
        double scale = moi.getAsDouble() / moiReference;
        armPivotController.setP(kP * MathUtil.clamp(scale, 0.5, 1.5));
        armPivotController.setI(kI * scale);
        armPivotController.setD(kD * Math.pow(scale, 0.7));

        inputs.position = Math.toDegrees(armPivotSim.getAngleRads());
        inputs.velocity = Math.toDegrees(armPivotSim.getVelocityRadPerSec());

        if (motionMagic) {
            double volts = MathUtil.clamp(armPivotController.calculate(inputs.position, setpoint) + ff.getAsDouble(), -12.0, 12.0);
            armPivotSim.setInputVoltage(volts);
            Logger.recordOutput("Arm/Pivot/Volts", volts);
        }
        armPivotLigament.setAngle(Math.toDegrees(armPivotSim.getAngleRads()));
        armPivotLigament.setLength(armPivotSim.getLengthMeters());

        Logger.recordOutput("Arm/Pivot/Setpoint", setpoint);
        Logger.recordOutput("Arm/Pivot/Position", inputs.position);
        Logger.recordOutput("Arm/Pivot/elevLength", armPivotSim.getLengthMeters());
        Logger.recordOutput("Arm/Pivot/armPivotVisual", armPivotVisual);

        Logger.recordOutput("Arm/Pivot/Debug Value", armPivotSim.getDebugVal());

        Logger.recordOutput("Arm/Pivot/PID", armPivotController.calculate(inputs.position, setpoint));
        Logger.recordOutput("Arm/Pivot/FF", ff.getAsDouble());
        Logger.recordOutput("Arm/Pivot/Scale", scale);
        Logger.recordOutput("Arm/Pivot/MOI", moi.getAsDouble());
    }
}
