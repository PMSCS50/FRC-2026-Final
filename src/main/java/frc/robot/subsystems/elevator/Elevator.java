package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.misc.VirtualPD;
import frc.robot.util.tunable.TunableControls.*;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorOne = new TalonFX(ElevatorConstants.elevatorOneCanId);
    private final TalonFX elevatorTwo = new TalonFX(ElevatorConstants.elevatorTwoCanId);
    private final TalonFX elevatorThree = new TalonFX(ElevatorConstants.elevatorThreeCanId);
    

    //Bullshit controls made out my ass
    private final TunableControlConstants elevatorControlConstants = 
        new TunableControlConstants(
            "ElevatorControls",
            new ControlConstants()
            .withPID(3, 0.0, 0.02)
            .withFeedforward(0.115, 0.0)
            .withPhysical(0.25, 0.1)
            .withTolerance(0.02)
        );

    private final TalonFXConfiguration config = elevatorControlConstants.getTalonFXConfiguration(true);

    private final MotionMagicVoltage positionRequest;

    private final StatusSignal<Angle> elevatorPosition;
    private final StatusSignal<AngularVelocity> elevatorVelocity;
    private final StatusSignal<AngularAcceleration> elevatorAcceleration;
    private final StatusSignal<Voltage> elevatorAppliedVoltage;
    private final StatisSignal<Boolean> atSetpoint;


    public Elevator() {
        configureMotors();

        //Elevator motors 2 and 3 are slaves to motor 1
        Follower slave = new Follower(elevatorOne.getDeviceID(), MotorAlignmentValue.Aligned);
        elevatorTwo.setControl(slave);
        elevatorThree.setControl(slave);

        positionRequest = new PositionVoltage(0.0).withSlot(0);

        VirtualPD.registerMotor(elevatorOne.getSupplyCurrent().asSupplier(), "Elevator");
        VirtualPD.registerMotor(elevatorTwo.getSupplyCurrent().asSupplier(), "Elevator");
        VirtualPD.registerMotor(elevatorThree.getSupplyCurrent().asSupplier(), "Elevator");

        elevatorPosition = elevatorOne.getPosition();
        elevatorVelocity = elevatorOne.getRotorVelocity();
        elevatorAcceleration = elevatorOne.getAcceleration();
        elevatorAppliedVoltage = elevatorOne.getMotorVoltage();
        atSetpoint = elevatorOne.getMotionMagicAtTarget();

        PhoenixUtil.registerStatusSignals(
            elevatorPosition,
            elevatorVelocity,
            elevatorAcceleration,
            elevatorAppliedVoltage,
            atSetpoint
        );
    }

    private void configureMotors() {
        
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralMode.Brake;
        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        applyConfig();
    }

    private void updateSlot0Configs() {
        config.slot0 = elevatorControlConstants.getSlot0Configs();
        applyConfig();
    }

    private void applyConfig() {
        elevatorOne.getConfigurator().apply(config);
        elevatorTwo.getConfigurator().apply(config);
        elevatorThree.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        if (elevatorControlConstants.hasChanged()) {
            updateSlot0Configs();
        }
    }

    public void goToPosition(double positionMeters) {
        elevatorOne.setControl(positionRequest.withPosition(positionMeters / ElevatorConstants.elevator_POSITION_COEFFICIENT));
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        elevatorOne.setNeutralMode(neutralMode);
        elevatorTwo.setNeutralMode(neutralMode);
        elevatorThree.setNeutralMode(neutralMode);
    }

    public void setDutyCycle(double dutyCycle) {
        elevatorOne.setControl(dutyCycleOut.withOutput(dutyCycle));
    }

    public boolean isAtSetpoint() {
        return atSetpoint.getValue();
    }

    public void stop() {
        elevatorOne.stopMotor();
    }

}