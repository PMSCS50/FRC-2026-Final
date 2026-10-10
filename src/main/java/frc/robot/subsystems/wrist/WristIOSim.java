package frc.robot.subsystems.wrist;

public class WristIOSim implements WristIO {
    // | Currently blank until I commit

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.position = 0.0;
        inputs.velocity = 0.0;
        inputs.acceleration = 0.0;
        inputs.appliedVoltage = 0.0;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        // Simulate setting duty cycle
    }

    @Override
    public void setMotionMagicPosition(double positionDegrees) {
        // Simulate setting motion magic position
    }

    @Override
    public void stop() {
        // Simulate stopping the wrist
    }
    
}
