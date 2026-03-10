package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    private final SparkMax pivotMotor = new SparkMax(IntakeConstants.pivotMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    private final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private final SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeMotorCanId, MotorType.kBrushless);

    //for starting the intake
    private final Timer initTimer = new Timer();
    public boolean initializing = false;
    
    public Intake() {
        pivotMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40);

        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40);

        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
    }

    @Override
    public void periodic() {

    }

    public void initinitIntake() {
        initTimer.restart();
        initializing = true;
    }

    //When the intake hits the bumper, the stalled motor will use a huge amount of current (probably reaching 40a limit).
    // ^ no? I don't think so
    
    //So, we can just check if the current usage exceeds 30a and then stop the motor.



    
    public void initIntake() {
        if (!initializing) return;
             
        double ampThreshold = 30;  
        double timeoutSeconds = 2.0;

        double current = pivotMotor.getOutputCurrent();

        if (initTimer.hasElapsed(timeoutSeconds)) {
            pivotMotor.set(0);
            initializing = false;
            return;
        }

        if (current < ampThreshold) {
            pivotMotor.set(IntakeConstants.pivotPower);
        } else {
            pivotMotor.set(0);
            pivotEncoder.setPosition(0);
            initializing = false;
        }
    }
    
    public void outitIntake() {
        if (!initializing) return;
             
        double ampThreshold = 30;  
        double timeoutSeconds = 2.0;

        double current = pivotMotor.getOutputCurrent();

        if (initTimer.hasElapsed(timeoutSeconds)) {
            pivotMotor.set(0);
            initializing = false;
            return;
        }

        if (current < ampThreshold) {
            pivotMotor.set(-IntakeConstants.pivotPower);
        } else {
            pivotMotor.set(0);
            pivotEncoder.setPosition(0);
            initializing = false;
        }
    }
    
    public void spinIntake(double speed) {
        intakeMotor.set(speed); 
    }
    
    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    public void deployIntake(double speed) {
        pivotMotor.set(speed);
    }

 
}