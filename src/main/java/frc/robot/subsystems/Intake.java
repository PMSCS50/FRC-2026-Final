package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final SparkClosedLoopController intakeClosedLoopController;

    private final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private final SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    // *calculations for freespinning neo
    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final double kIntakeMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;

    // *for starting the intake
    public boolean initializing = false;

    public Intake() {
        intakeClosedLoopController = intakeMotor.getClosedLoopController();
 
        intakeMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40)
                .secondaryCurrentLimit(60)
                .closedLoopRampRate(.3);
        intakeMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // .pid(0.0025, 0, .1) // p = 0.01 pulses // d = .01 seems alright
                .pid(.01, 0, 0)
                .outputRange(-.95, .95).feedForward.kV(.6); // current value = .00017618

        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake/Amperage (amps)", intakeMotor.getOutputCurrent());
        Logger.recordOutput("Intake/Velocity (rpm)", intakeEncoder.getVelocity());

        //SmartDashboard.putNumber("Intake/Amps", intakeMotor.getOutputCurrent());
    }

    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    public void spinIntakePID(double percent) {
        double targetRPM = 5676 * percent;
        intakeClosedLoopController.setSetpoint(targetRPM, ControlType.kVelocity);
    }

    public void spinIntakeDuty(double speed) {
        intakeMotor.set(speed);
    }

    public RelativeEncoder getIntakeEncoder() {
        return intakeEncoder;
    }

    public SparkMax getIntakeMotor() {
        return intakeMotor;
    }
}