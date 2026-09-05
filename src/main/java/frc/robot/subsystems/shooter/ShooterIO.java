package frc.robot.subsystems.shooter;

public interface ShooterIO {
    public static class ShooterIOInputs {
        public double totalCurrent;
        public double shooterVelocity;
    }

    public void updateInputs(ShooterIOInputs inputs);
    public double rpsFromDistanceRegression(double distance);
    public void rpsControl(double distance);
    public void stop();
    public void spinKickersMax();
    public void spinKickersSpecified(double percent);
    public boolean atCorrectRPS(double distance);
}
