package frc.robot.subsystems.telescoping_arm.wrist;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.PhoenixUtil;

public class WristIOReal implements WristIO {
    private final TalonFX wristMotor = new TalonFX(ArmConstants.wristMotorCanId);

    private final TalonFXConfiguration config;
    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    private final StatusSignal<Angle> pos = wristMotor.getPosition();
    private final StatusSignal<AngularVelocity> vel = wristMotor.getRotorVelocity();

    public WristIOReal() {
        // |Configs pulled out of Suhas's ass
        config = new TalonFXConfiguration();
        config.Slot0.kP = ArmConstants.wristkP;
        config.Slot0.kI = ArmConstants.wristkI;
        config.Slot0.kD = ArmConstants.wristkD;

        wristMotor.getConfigurator().apply(config);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        PhoenixUtil.registerStatusSignals();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.position = pos.getValueAsDouble();
        inputs.velocity = vel.getValueAsDouble();
    }

    @Override
    public void setDutyCycle(double dc) {
        wristMotor.setControl(dutyCycle.withOutput(dc));
    }

    @Override
    public void setMotionMagicPosition(double positionDegrees) {
        wristMotor.setControl(mmRequest.withPosition(positionDegrees));
    }

    @Override
    public void stop() {
        wristMotor.setControl(dutyCycle.withOutput(0));
    }
}
