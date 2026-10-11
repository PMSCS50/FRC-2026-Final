package frc.robot.subsystems.telescoping_arm.pivot;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivot extends SubsystemBase {
    private final ArmPivotIO io;
    private final ArmPivotIOInputsAutoLogged inputs = new ArmPivotIOInputsAutoLogged();

    public ArmPivot(ArmPivotIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("LoggedPivot (Arm)", inputs);

    }

    public void goToPosition(double positionMeters) {
        io.setMotionMagicPosition(positionMeters);
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        io.setNeutralMode(neutralMode);
    }

    public void setDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }

    public boolean isAtSetpoint() {
        return inputs.atSetpoint;
    }

    public void stop() {
        io.stop();
    }


    
}
