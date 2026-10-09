package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {

    private final TalonFX elevatorOne = new TalonFX(ElevatorConstants.elevatorOneCanId);
    private final TalonFX elevatorTwo = new TalonFX(ElevatorConstants.elevatorTwoCanId);
    private final TalonFX elevatorThree = new TalonFX(ElevatorConstants.elevatorThreeCanId);

    private final TalonFXConfiguration config;
    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

    private final StatusSignal<?> pos = elevatorOne.getPosition();
    private final StatusSignal<?> vel = elevatorOne.getRotorVelocity();
    private final StatusSignal<?> acc = elevatorOne.getAcceleration();
    private final StatusSignal<?> volts = elevatorOne.getMotorVoltage();
    private final StatusSignal<?> atSetpoint = elevatorOne.getMotionMagicAtTarget();

    public ElevatorIOReal(TalonFXConfiguration cfg) {
        this.config = cfg;

        elevatorOne.getConfigurator().apply(config);
        elevatorTwo.getConfigurator().apply(config);
        elevatorThree.getConfigurator().apply(config);

        elevatorTwo.setControl(new Follower(elevatorOne.getDeviceID(), MotorAlignmentValue.Aligned));
        elevatorThree.setControl(new Follower(elevatorOne.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        pos.refresh();
        vel.refresh();
        acc.refresh();
        volts.refresh();
        atSetpoint.refresh();

        inputs.position = (Angle) pos.getValue();
        inputs.velocity = (AngularVelocity) vel.getValue();
        inputs.acceleration = (AngularAcceleration) acc.getValue();
        inputs.appliedVoltage = (Voltage) volts.getValue();
        inputs.atSetpoint = (Boolean) atSetpoint.getValue();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        elevatorOne.setControl(dutyRequest.withOutput(dutyCycle));
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
    public void setNeutralMode(Object neutralMode) {
        elevatorOne.setNeutralMode((NeutralModeValue) neutralMode);
        elevatorTwo.setNeutralMode((NeutralModeValue) neutralMode);
        elevatorThree.setNeutralMode((NeutralModeValue) neutralMode);
    }
}
