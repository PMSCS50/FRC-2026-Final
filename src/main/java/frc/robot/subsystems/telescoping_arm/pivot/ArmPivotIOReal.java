package frc.robot.subsystems.telescoping_arm.pivot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.misc.VirtualPD;

public class ArmPivotIOReal implements ArmPivotIO {

    private final TalonFX pivotOne = new TalonFX(ArmConstants.pivotOneCanId);
    private final TalonFX pivotTwo = new TalonFX(ArmConstants.pivotTwoCanId);
    private final TalonFX pivotThree = new TalonFX(ArmConstants.pivotThreeCanId);

    private final TalonFXConfiguration config;
    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

    private final StatusSignal<Angle> pos = pivotOne.getPosition();
    private final StatusSignal<AngularVelocity> vel = pivotOne.getRotorVelocity();
    private final StatusSignal<AngularAcceleration> acc = pivotOne.getAcceleration();
    private final StatusSignal<Voltage> volts = pivotOne.getMotorVoltage();
    private final StatusSignal<Boolean> atSetpoint = pivotOne.getMotionMagicAtTarget();

    public ArmPivotIOReal() {
        this.config = new TalonFXConfiguration();

        //Just initiliazing config here.
        // |Numbers pulled out of Suhas's ass
        config.Slot0.withKP(3)
                    .withKI(0.0)
                    .withKD(0.02)
                    .withKS(0.115)
                    .withKV(0.0)
                    .withKA(0.25)
                    .withKG(0.1);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;


        pivotOne.getConfigurator().apply(config);
        pivotTwo.getConfigurator().apply(config);
        pivotThree.getConfigurator().apply(config);

        //Good slaves obey their master
        Follower goodSlave = new Follower(pivotOne.getDeviceID(), MotorAlignmentValue.Aligned);
        pivotTwo.setControl(goodSlave);
        pivotThree.setControl(goodSlave);

        registerMotors();
        PhoenixUtil.registerStatusSignals(
            pos,
            vel,
            acc,
            volts,
            atSetpoint
        );
    }

    @Override
    public void registerMotors() {
        VirtualPD.registerMotor(pivotOne.getStatorCurrent().asSupplier(), "Pivot (Arm)");
        VirtualPD.registerMotor(pivotTwo.getStatorCurrent().asSupplier(), "Pivot (Arm)");
        VirtualPD.registerMotor(pivotThree.getStatorCurrent().asSupplier(), "Pivot (Arm)");
    }

    @Override
    public void updateInputs(ArmPivotIOInputs inputs) {
        inputs.position = pos.getValueAsDouble();
        inputs.velocity = vel.getValueAsDouble();
        inputs.atSetpoint = atSetpoint.getValue();
    }

    @Override
    public void setMotionMagicPosition(double position) {
        pivotOne.setControl(mmRequest.withPosition(position * ArmConstants.pivotGearing));
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        pivotOne.setNeutralMode(neutralMode);
        pivotTwo.setNeutralMode(neutralMode);
        pivotThree.setNeutralMode(neutralMode);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        pivotOne.setControl(dutyRequest.withOutput(dutyCycle));
    }

    @Override
    public void stop() {
        setDutyCycle(0.0);
    }


}
