package frc.robot.subsystems.telescoping_arm.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public double position;
        public double velocity;
        public double acceleration;
        public double appliedVoltage;
    }

    void updateInputs(WristIOInputs inputs);
    void setDutyCycle(double dutyCycle);
    void setMotionMagicPosition(double positionDegrees);
    void stop();
}
