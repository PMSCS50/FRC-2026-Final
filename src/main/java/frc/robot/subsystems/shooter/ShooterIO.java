package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double shooterVelocity;
        public double kickerVelocity;
    }

    public void registerMotors();
    public void updateInputs(ShooterIOInputs inputs);
    public void setShooterVoltage(double volts);
    public void setKickerVoltage(double volts);

    public void setShooterVelocity(double rps);
}
