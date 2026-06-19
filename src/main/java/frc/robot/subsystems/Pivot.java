package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.Logger;
//import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private final SparkClosedLoopController pivotClosedLoopController;

    private final SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    private final SparkMax pivotMotor = new SparkMax(IntakeConstants.pivotMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    private double outputMin = -1, outputMax = 1;
    private double targetPosition = 0;

    // *MAXMotion profile constraints.
    private static final double kMaxMotionMaxVelocity = 1702.8;
    private static final double kMaxMotionMaxAcceleration = 5000;
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
                .secondaryCurrentLimit(60)
                .closedLoopRampRate(1);

        pivotMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(3, 0, 0)
                .outputRange(outputMin, outputMax)
                .positionWrappingEnabled(false)
                .feedForward         
                .kS(0.0)
                .kCos(0.2)
                .kCosRatio(0.375 / 17);
;

        // *MaxMotion config
        pivotMotorConfig.closedLoop.maxMotion
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .cruiseVelocity(kMaxMotionMaxVelocity)
                .maxAcceleration(kMaxMotionMaxAcceleration)
                .allowedProfileError(kMaxMotionAllowedErrorRotations);

        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        double pivotAmps = getPivotAmps();
        boolean overcurrent = pivotAmps > IntakeConstants.kPivotStallCurrent;
        boolean stalledRising = stallDebouncer.calculate(overcurrent);

        if (stalledRising) {
            pivotStalled = !unstallDebouncer.calculate(!overcurrent);
        } else {
            unstallDebouncer.calculate(false); // keep it reset
            pivotStalled = false;
        }
        
        Logger.recordOutput("Pivot/Amperage (amps)", pivotAmps);
        Logger.recordOutput("Pivot/Pivot Amount (rotations)", getPivotEncoder().getPosition());
        Logger.recordOutput("Pivot/Is Stalled", pivotStalled);

        Logger.recordOutput("Pivot/TargetPosition", targetPosition);
        // Logger.recordOutput("Pivot/PositionError", targetPosition - getPivotPosition());
        // Logger.recordOutput("Pivot/AppliedOutput", pivotMotor.getAppliedOutput());
        Logger.recordOutput("Pivot/Velocity", getPivotVelocity());
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
            pivotClosedLoopController.setSetpoint(targetRPM, ControlType.kVelocity);
        }
    }

    public void spinPivotPID(double percent) {
        double targetRPM = 5676 * percent;
        pivotClosedLoopController.setSetpoint(targetRPM, ControlType.kVelocity);
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
        targetPosition = targetRotations;

        pivotClosedLoopController.setSetpoint(
            targetRotations,
            ControlType.kMAXMotionPositionControl
        );
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

    // public void setOutputLimits(double speed) {
    //     outputMin = -speed;
    //     outputMax = speed;
    // }

    // public void setTopLimit(double speed) {
    //     outputMax = speed;
    // }

    // public void setBottomLimit(double speed) {
    //     outputMin = -speed;
    // }
}