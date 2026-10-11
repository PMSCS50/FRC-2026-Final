package frc.robot.subsystems.telescoping_arm.pivot;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

public interface ArmPivotIO {
    @AutoLog
    public static class ArmPivotIOInputs {
        public double position = 0.0;
        public double velocity = 0.0;
        public boolean atSetpoint = false;
    }

    public void updateInputs(ArmPivotIOInputs inputs);
    public void registerMotors();
    public void setMotionMagicPosition(double position);
    public void setNeutralMode(NeutralModeValue neutralMode);
    public void setDutyCycle(double dutyCycle);
    public void stop();
    
}
