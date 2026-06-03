package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private final SparkClosedLoopController pivotClosedLoopController;

    private final SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    private final SparkMax pivotMotor = new SparkMax(IntakeConstants.pivotMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    private double outputMin = -0.3, outputMax = 0.3;

    // *MAXMotion profile constraints.
    private static final double kMaxMotionMaxVelocityRpm        = 1702.8;
    private static final double kMaxMotionMaxAccelerationRpmSec = 2000;
    private static final double kMaxMotionAllowedErrorRotations = 0.05;

    private final Debouncer stallDebouncer = new Debouncer(0.1, DebounceType.kRising);
    private final Debouncer unstallDebouncer = new Debouncer(0.6, DebounceType.kFalling);
    public boolean pivotStalled = false;

    public static final double kIntakeMotorFreeSpeedRps = IntakeConstants.kPivotFreeSpeedRpm / 60;

    // *for starting the intake
    public boolean initializing = false;

    public Pivot() {
        pivotEncoder.setPosition(0);
        pivotClosedLoopController = pivotMotor.getClosedLoopController();

        pivotMotorConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .closedLoopRampRate(1);

        pivotMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(.05, 0, 0)
                .outputRange(outputMin, outputMax)
                .positionWrappingEnabled(false);

        // *MaxMotion config
        pivotMotorConfig.closedLoop.maxMotion
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .cruiseVelocity(kMaxMotionMaxVelocityRpm)
                .maxAcceleration(kMaxMotionMaxAccelerationRpmSec)
                .allowedProfileError(kMaxMotionAllowedErrorRotations);

        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        boolean overcurrent = getPivotAmps() > IntakeConstants.kPivotStallCurrent;
        pivotStalled = stallDebouncer.calculate(overcurrent) && !unstallDebouncer.calculate(!overcurrent);
        Logger.recordOutput("Subsystems/Pivot/isStalled", pivotStalled);
        Logger.recordOutput("Subsystems/Pivot/amps", getPivotAmps());

        Logger.recordOutput("Subsystems/Pivot/Is Stalled", pivotStalled);
    }

    public double getPivotAmps() {
        return pivotMotor.getOutputCurrent();
    }

    public boolean pivotIsStalled() {
        return pivotStalled;
    }

    public void spinPivotPIDAmped(double percent) {
        double targetRPM = 5676 * percent;
        if (!pivotIsStalled()) {
            pivotClosedLoopController.setSetpoint(targetRPM, ControlType.kPosition);
        }
    }

    public void spinPivotPID(double percent) {
        double targetRPM = 5676 * percent;
        pivotClosedLoopController.setSetpoint(targetRPM, ControlType.kPosition);
    }

    public void spinPivotDutyAmped(double speed) {
        if (!pivotIsStalled()) {
            pivotMotor.set(speed);
        } else {
            pivotMotor.stopMotor();
        }
    }

    public void spinPivotDuty(double speed) {
        pivotMotor.set(speed);
    }

    public SparkMaxConfig getPivotMotorConfig() {
        return pivotMotorConfig;
    }

    public SparkMax getPivotMotor() {
        return pivotMotor;
    }

    public void stopPivot() {
        pivotMotor.stopMotor();
    }

    public RelativeEncoder getPivotEncoder() {
        return pivotEncoder;
    }

    public double getPivotVelocity() {
        return pivotEncoder.getVelocity();
    }

    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }

    public void resetPivot() {
        pivotEncoder.setPosition(0);
    }

    // *Plain position control — no profiling
    public void goToPosition(double targetRotations) {
        pivotClosedLoopController.setSetpoint(targetRotations, ControlType.kPosition);
    }

    // *MAXMotion position control — trapezoidal profile, use this for deployment
    public void goToPositionMAXMotion(double targetRotations) {
        pivotClosedLoopController.setSetpoint(targetRotations, ControlType.kMAXMotionPositionControl);
    }

    // *MAXMotion with stall guard
    public void goToPositionMAXMotionAmped(double targetRotations) {
        if (!pivotIsStalled()) {
            pivotClosedLoopController.setSetpoint(targetRotations, ControlType.kMAXMotionPositionControl);
        }
    }

    public boolean atPosition(double targetRotations, double toleranceRotations) {
        return Math.abs(getPivotPosition() - targetRotations) < toleranceRotations;
    }

    public void setOutputLimits(double speed) {
        outputMin = -speed;
        outputMax = speed;
    }

    public void setTopLimit(double speed) {
        outputMax = speed;
    }

    public void setBottomLimit(double speed) {
        outputMin = -speed;
    }
}