package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    
    @AutoLog
    public static class PivotIOInputs {
        public double motorVoltage;
        public double motorPosition;
        public double motorVelocity;
    }

    public void setVoltage(double volts);
    public void setDutyCycle(double duty);
    public void setPosition(double rotations); // for PIDs
    public void setEncoderPosition(double pos);
    public void registerMotors();
    
    public void updateInputs(PivotIOInputs inputs);
}
