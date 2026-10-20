package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.util.misc.VirtualPD;
import frc.robot.util.tunable.TunableControls.*;

public class PivotIOReal implements PivotIO {
    private final SparkMax motor = new SparkMax(IntakeConstants.pivotMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    TunablePIDController pidController = new TunablePIDController(
        new TunableControlConstants(
            "PivotControls", 
            new ControlConstants()
            .withPID(0.1, 0, 0)
            .withTolerance(1.0))
    );

    private final SparkClosedLoopController closedLoop = motor.getClosedLoopController();

    public PivotIOReal() {
        motorConfig.inverted(true)
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(40)
              .closedLoopRampRate(1);

        motorConfig.closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .pid(
                pidController.getPID()[0], 
                pidController.getPID()[1], 
                pidController.getPID()[2])
              .outputRange(-0.3, 0.3)
              .positionWrappingEnabled(false)
            //   .feedForward
            //   .kCos(0.15)
            //   .kCosRatio(0.375 / IntakeConstants.kPivotSetpointB)
                ;

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void updatePID(double[] pid) {
        motorConfig.closedLoop
        .pid(pid[0], pid[1], pid[2]);
    }

    @Override
    public void registerMotors() {
        VirtualPD.registerMotor(() -> Amps.of(motor.getOutputCurrent()), "Pivot");
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
        pidController.updateParams();
        updatePID(pidController.getPID());


        inputs.motorVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.motorPosition = encoder.getPosition();
        inputs.motorVelocity = encoder.getVelocity();

        Logger.recordOutput("Pivot/pid", pidController.getPID());
    }
}