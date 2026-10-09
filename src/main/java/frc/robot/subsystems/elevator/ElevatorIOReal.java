package frc.robot.subsystems.elevator;

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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.misc.PhoenixUtil;
import frc.robot.util.misc.VirtualPD;

public class ElevatorIOReal implements ElevatorIO {

    private final TalonFX elevatorOne = new TalonFX(ElevatorConstants.elevatorOneCanId);
    private final TalonFX elevatorTwo = new TalonFX(ElevatorConstants.elevatorTwoCanId);
    private final TalonFX elevatorThree = new TalonFX(ElevatorConstants.elevatorThreeCanId);

    private final TalonFXConfiguration config;
    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

    private final StatusSignal<Angle> pos = elevatorOne.getPosition();
    private final StatusSignal<AngularVelocity> vel = elevatorOne.getRotorVelocity();
    private final StatusSignal<AngularAcceleration> acc = elevatorOne.getAcceleration();
    private final StatusSignal<Voltage> volts = elevatorOne.getMotorVoltage();
    private final StatusSignal<Boolean> atSetpoint = elevatorOne.getMotionMagicAtTarget();

    public ElevatorIOReal() {
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


        elevatorOne.getConfigurator().apply(config);
        elevatorTwo.getConfigurator().apply(config);
        elevatorThree.getConfigurator().apply(config);

        Follower slave = new Follower(elevatorOne.getDeviceID(), MotorAlignmentValue.Aligned);
        elevatorTwo.setControl(slave);
        elevatorThree.setControl(slave);

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
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = pos.getValue();
        inputs.velocity = vel.getValue();
        inputs.acceleration = acc.getValue();
        inputs.appliedVoltage = volts.getValue();
        inputs.atSetpoint = atSetpoint.getValue();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        elevatorOne.setControl(dutyRequest.withOutput(dutyCycle));
    }

    private void registerMotors() {
        VirtualPD.registerMotor(elevatorOne.getStatorCurrent().asSupplier(), "Elevator");
        VirtualPD.registerMotor(elevatorTwo.getStatorCurrent().asSupplier(), "Elevator");
        VirtualPD.registerMotor(elevatorThree.getStatorCurrent().asSupplier(), "Elevator");
    }

    @Override
    public void setMotionMagicPosition(double positionMeters) {
        elevatorOne.setControl(mmRequest.withPosition(positionMeters / ElevatorConstants.ELEVATOR_POSITION_COEFFICIENT));
    }

    @Override
    public void stop() {
        elevatorOne.stopMotor();
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        elevatorOne.setNeutralMode(neutralMode);
        elevatorTwo.setNeutralMode(neutralMode);
        elevatorThree.setNeutralMode(neutralMode);
    }
}
