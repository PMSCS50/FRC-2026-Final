package frc.robot.subsystems.telescoping_arm.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double position;
        public boolean atSetpoint;
        public double extension;
    }

    void updateInputs(ElevatorIOInputs inputs);

    void setDutyCycle(double dutyCycle);

    void setMotionMagicPosition(double positionMeters);

    void stop();

    void setNeutralMode(NeutralModeValue neutralMode); // NeutralModeValue for real, ignored in sim
}
