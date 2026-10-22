package frc.robot.subsystems.telescoping_arm.elevator;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("LoggedArm (Elevator)", inputs);
    }

    public void goToPosition(double positionMeters) {
        io.setMotionMagicPosition(positionMeters);
    }

    public double getPositionMeters() {
        return inputs.extension;
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        io.setNeutralMode(neutralMode);
    }

    public void setDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }

    public boolean isAtSetpoint() {
        return inputs.atSetpoint;
    }

    public void stop() {
        io.stop();
    }

}