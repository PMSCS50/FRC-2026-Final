package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double motorVoltage;
        public double motorVelocity;
    }

    public void updateInputs(IntakeIOInputs inputs);

    public void setVoltage(double volts);

    public void registerMotors();
}
