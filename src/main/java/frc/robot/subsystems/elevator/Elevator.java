package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.misc.VirtualPD;
import frc.robot.util.tunable.TunableControls.*;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorMotor1 = new TalonFX(ElevatorConstants.elevatorMotor1CanId);
    private final TalonFX elevatorMotor2 = new TalonFX(ElevatorConstants.elevatorMotor2CanId);
    private final TalonFX elevatorMotor3 = new TalonFX(ElevatorConstants.elevatorMotor3CanId);

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

    private TalonFXConfiguration config = elevatorControlConstants.getTalonFXConfiguration(false);

    private final PositionVoltage positionRequest;

    public Elevator() {
        
        configureMotors();

        //Elevator motors 2 and 3 are slaves to motor 1
        Follower slave = new Follower(elevatorMotor1.getDeviceID(), MotorAlignmentValue.Aligned);

        elevatorMotor2.setControl(slave);
        elevatorMotor3.setControl(slave);

        positionRequest = new PositionVoltage(0.0).withSlot(0);

        VirtualPD.registerMotor(elevatorMotor1.getSupplyCurrent().asSupplier(), "Elevator");
        VirtualPD.registerMotor(elevatorMotor2.getSupplyCurrent().asSupplier(), "Elevator");
        VirtualPD.registerMotor(elevatorMotor3.getSupplyCurrent().asSupplier(), "Elevator");
    }

    private void configureMotors() {
        config = elevatorControlConstants.getTalonFXConfiguration(false);

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        elevatorMotor1.getConfigurator().apply(config);
        elevatorMotor2.getConfigurator().apply(config);
        elevatorMotor3.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        if (elevatorControlConstants.hasChanged()) {
            configureMotors();
        }
    }

    //Currently only takes rotation. When Oliver furthers CAD, we can take elevator pos.
    public void goToPosition(double position) {
        elevatorMotor1.setControl(positionRequest.withPosition(30));
    }

    public void stop() {
        elevatorMotor1.stopMotor();
    }


}