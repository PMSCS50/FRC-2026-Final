package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public Angle position;
        public AngularVelocity velocity;
        public AngularAcceleration acceleration;
        public Voltage appliedVoltage;
        public boolean atSetpoint;
    }

    void updateInputs(ElevatorIOInputs inputs);

    void setDutyCycle(double dutyCycle);

    void setMotionMagicPosition(double positionMeters);

    void stop();

    void setNeutralMode(NeutralModeValue neutralMode); // NeutralModeValue for real, ignored in sim
}
