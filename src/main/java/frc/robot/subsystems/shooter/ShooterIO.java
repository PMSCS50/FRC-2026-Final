package frc.robot.subsystems.shooter;

public interface ShooterIO {
    public static class ShooterIOInputs {
        public double totalCurrent;
        public double shooterVelocity;
        public double kickerVelocity;
    }

    public void updateInputs(ShooterIOInputs inputs);
    public void setShooterVoltage(double volts);
    public void setKickerVoltage(double volts);

    public void setShooterVelocity(double rps);
}
