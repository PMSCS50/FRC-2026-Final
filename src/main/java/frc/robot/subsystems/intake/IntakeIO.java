package frc.robot.subsystems.intake;


public interface IntakeIO {
    public static class IntakeIOInputs {
        public double motorVoltage;
        public double motorVelocity;
    }

    public void updateInputs(IntakeIOInputs inputs);

    public void setVoltage(double volts);

    public void registerMotors();
}
