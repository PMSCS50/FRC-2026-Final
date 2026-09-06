package frc.robot.subsystems.pivot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IntakeConstants;

public class PivotIOReal implements PivotIO {
    private final SparkMax motor = new SparkMax(IntakeConstants.intakeMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    private final SparkClosedLoopController closedLoop = motor.getClosedLoopController();

    public PivotIOReal() {
        motorConfig.inverted(true)
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(40)
              .closedLoopRampRate(1);

        motorConfig.closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .pid(0.4, 0.4, 0.0)
              .outputRange(-0.3, 0.3)
              .positionWrappingEnabled(false)
              .feedForward
              .kCos(0.15)
              .kCosRatio(0.375 / IntakeConstants.kPivotSetpointB);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setDutyCycle(double duty) {
        motor.set(duty);
    }

    @Override
    public void setPosition(double rotations) {
        closedLoop.setSetpoint(rotations, ControlType.kPosition);
    }

    @Override
    public void setEncoderPosition(double pos) {
        encoder.setPosition(pos);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.motorAmperage = motor.getOutputCurrent(); // rotations or radians
        inputs.motorVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.motorPosition = encoder.getPosition();
        inputs.motorVelocity = encoder.getVelocity();
    }
}