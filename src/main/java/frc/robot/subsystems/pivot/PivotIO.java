package frc.robot.subsystems.pivot;

public interface PivotIO {
    public static class PivotIOInputs {
        public double motorAmperage;
        public double motorVoltage;
        public double motorPosition;
        public double motorVelocity;
    }

    public void setVoltage(double volts);
    public void setDutyCycle(double duty);
    public void setPosition(double rotations); // for PIDs
    public void setEncoderPosition(double pos);
    
    public void updateInputs(PivotIOInputs inputs);
}
