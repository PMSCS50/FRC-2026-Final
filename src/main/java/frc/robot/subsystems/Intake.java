package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final SparkClosedLoopController pivotClosedLoopController;

    private final SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    private final SparkMax pivotMotor = new SparkMax(IntakeConstants.pivotMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    private final SparkClosedLoopController intakeClosedLoopController;

    private final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private final SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    
    // calculations for freespinning neo
    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
    public static final double kIntakeMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    


// Add to pivotMotorConfig in Intake constructor — replace current closedLoop config:


// Add this method to Intake.java:


    //for starting the intake
    private final Timer initTimer = new Timer();
    public boolean initializing = false;
    
    public Intake() {
        pivotEncoder.setPosition(0);
        pivotClosedLoopController = pivotMotor.getClosedLoopController();
        intakeClosedLoopController = intakeMotor.getClosedLoopController();
        pivotMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40).closedLoopRampRate(1);
        pivotMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(10, 0, 0) 
            .outputRange(-0.5, 0.5)  
            .positionWrappingEnabled(false);


        
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40).closedLoopRampRate(1);
        intakeMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // .pid(0.0025, 0, .1) // p = 0.01 pulses // d = .01 seems alright
            .pid(0, 0, 0)
            .outputRange(-1, 1)
            .feedForward.kV(.0021); // current value = .00017618
            

        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }


    @Override
    public void periodic() {}

    public void stopIntake() {
        intakeMotor.stopMotor();
    }
    public void stopPivot() {
        pivotMotor.stopMotor();
    }
    public void stopAll() {
        intakeMotor.stopMotor();
        pivotMotor.stopMotor();
    }

    public void spinIntakePID(double percent) {
        double targetRPM = 5676 * percent;
        intakeClosedLoopController.setSetpoint(targetRPM, ControlType.kVelocity);
    }
    public void spinIntakePIDTimed(double percent, double time) {
        double targetRPM = 5676 * percent;
        intakeClosedLoopController.setSetpoint(targetRPM, ControlType.kVelocity);
    }

    public void spinPivotPID(double percent) {
        double targetRPM = 5676 * percent;
        pivotClosedLoopController.setSetpoint(targetRPM, ControlType.kPosition);
    }

    public void spinIntakeDuty(double speed) {
        intakeMotor.set(speed); 
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

    public RelativeEncoder getPivotEncoder() {
        return pivotEncoder;
    }

    public RelativeEncoder getIntakeEncoder() {
        return intakeEncoder;
    }
    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }

    public void resetPivot() {
        pivotEncoder.setPosition(0);
    }

    public void goToPosition(double targetRotations) {
        pivotClosedLoopController.setSetpoint(targetRotations, ControlType.kPosition);
    }

    public boolean atPosition(double targetRotations, double toleranceRotations) {
        return Math.abs(getPivotPosition() - targetRotations) < toleranceRotations;
    }



    // public SparkMaxConfig getIntakeMotorConfig() {
    //     return intakeMotorConfig;
    // }

    // public SparkMax getIntakeMotor() {
    //     return intakeMotor;
    // }

    // public Timer getInitTimer() {
    //     return initTimer;
    // }

    // public boolean isInitializing() {
    //     return initializing;
    // }

    // public void setInitializing(boolean initializing) {
    //     this.initializing = initializing;
    // }
    


 
}