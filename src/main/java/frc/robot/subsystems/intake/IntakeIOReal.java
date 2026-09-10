package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IntakeConstants;
import frc.robot.util.misc.VirtualPD;

public class IntakeIOReal implements IntakeIO {

    private final SparkMax motor = new SparkMax(IntakeConstants.intakeMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    public IntakeIOReal() {
        motorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40)
                .secondaryCurrentLimit(60)
                .closedLoopRampRate(.3);
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // .pid(0.0025, 0, .1) // p = 0.01 pulses // d = .01 seems alright
                .pid(.01, 0, 0)
                .outputRange(-.95, .95).feedForward.kV(.6); // current value = .00017618

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void registerMotors() {
        VirtualPD.registerMotor(() -> Amps.of(motor.getOutputCurrent()), "Intake");
    }

    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motorVoltage = motor.getBusVoltage();
        inputs.motorVelocity = encoder.getVelocity();

    }
}
