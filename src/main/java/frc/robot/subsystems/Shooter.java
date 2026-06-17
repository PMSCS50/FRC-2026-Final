package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.vision.VisionGeneral;

public class Shooter extends SubsystemBase {

    // !MOTORS
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final SparkMax kicker1 = new SparkMax(ShooterConstants.kickerMotorCanId1, MotorType.kBrushless);
    private final SparkMax kicker2 = new SparkMax(ShooterConstants.kickerMotorCanId2, MotorType.kBrushless);

    private final VisionGeneral vision;

    // !MOTOR CONTROLS
    // *DutyCycleOut: simple 0-1 power control, used by setShooterSpeed()
    private final DutyCycleOut motorControl = new DutyCycleOut(0.0);

    // *VelocityVoltage: closed-loop RPM control, used by setVelocityTo()
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);

    // !SHOOTER PHYSICS CONSTANTS
    private double velocity = 0.0;
    private final double shooterAngle = 70.0;                  // degrees
    private final double phi = Math.toRadians(shooterAngle);   // radians
    private final double shooterHeight = 0.508;                // meters from ground
    
    // !CONSTRUCTOR
    public Shooter(VisionGeneral vision) {
        this.vision = vision;
        shooterMotor1 = new TalonFX(ShooterConstants.shooterMotorCanId1);
        shooterMotor2 = new TalonFX(ShooterConstants.shooterMotorCanId2);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        configureShooterMotor(shooterConfig);

        shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterMotor1.getConfigurator().apply(shooterConfig);

        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterMotor2.getConfigurator().apply(shooterConfig);
        shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed));

        // *Kicker config
        SparkMaxConfig kickerConfig1 = new SparkMaxConfig();
        SparkMaxConfig kickerConfig2 = new SparkMaxConfig();
        
        kickerConfig1
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40);
        kickerConfig2
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .follow(kicker1, true);

        kicker1.configure(kickerConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kicker2.configure(kickerConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // *Applies shared TalonFX settings — PID gains, current limits, neutral mode
    private void configureShooterMotor(TalonFXConfiguration config) {
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // *Velocity PID gains — tune these on the real robot
        config.Slot0.kS = 0;   // static friction compensation (volts)
        config.Slot0.kV = 0.1150;  // velocity feedforward
        config.Slot0.kP = .2; // .8   // proportional
        config.Slot0.kI = 0;     // integral
        config.Slot0.kD = 0;     // derivative
    }

    // !PERIODIC
    @Override
    public void periodic()   
    {
        // SmartDashboard.putNumber("Shooter RPM", shooterMotor1.getVelocity().getValueAsDouble() * 60.0);
        Logger.recordOutput("Shooter/Amperage (amps)",shooterMotor1.getMotorStallCurrent().getValueAsDouble());
        Logger.recordOutput("Shooter/Velocity (rps)", shooterMotor1.getVelocity().getValueAsDouble());
    }

    // !SHOOTING METHODS
    //*Simple duty cycle control (0.0 to 1.0).
    // ?Used for manual testing from RobotContainer buttons. Also runs kickers at full power.
    public void setShooterSpeed(double speed) {
        shooterMotor1.setControl(motorControl.withOutput(speed));
        kicker1.set(1);
        // SmartDashboard.putNumber("Actual RPS", shooterMotor1.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Motor Output", shooterMotor1.getMotorVoltage().getValueAsDouble());
    }

    // *Regression model by Kevin
    public double rpsFromDistanceRegression(double distance) {
        double rps = 0.1322042143 * Math.pow(distance, 4)
                   - 1.110063156  * Math.pow(distance, 3)
                   + 3.621489461  * Math.pow(distance, 2)
                   + 0.1849702218 * distance
                   + 33.86388054;
        return rps;
    }

    // *Sets shooter velocity based on distance to target, using the regression model. Also runs kickers at full power.
    public void rpsControl(double distance) {
        double rps = this.rpsFromDistanceRegression(distance);
        shooterMotor1.setControl(velocityRequest.withVelocity(rps));
        // SmartDashboard.putNumber("Target RPM", rpm);
        Logger.recordOutput("Shooter/rpsControl/Target RPS", rps);
        Logger.recordOutput("Shooter/rpsControl/Actual RPS", shooterMotor1.getVelocity().getValueAsDouble());
    }

    // *Runs the kicker motors at full power to feed balls into the shooter.
    public void spinKickersMax() {
        kicker1.set(1);
    }

    // *Runs the kicker motors at a specified speed.
    public void spinKickersSpecified(double speed) {
        kicker1.set(speed);
    }

    // *Checks if the shooter is within a certain RPM threshold of the target RPM based on current distance to target.
    public boolean atCorrectRPS() {
        double currentRPS = shooterMotor1.getVelocity().getValueAsDouble();
        double targetRPS = this.rpsFromDistanceRegression(vision.getDistanceToTarget(vision.cachedHubPose)); 

        Logger.recordOutput("Shooter/atCurrentRPS/Current RPS", currentRPS);
        Logger.recordOutput("Shooter/atCurrentRPS/Target RPS", targetRPS);
        Logger.recordOutput("Shooter/atCurrentRPS/RPS Difference", Math.abs(currentRPS - targetRPS));

        return Math.abs(currentRPS - targetRPS) < 5.0;
    }

    // *Same as atCorrectRPM but with a tighter threshold for more precise shooting.
    // ?Useful for testing and tuning the regression model and PID gains.
    public boolean atCorrectRPSFixed(double distance) {
        double currentRPS = shooterMotor1.getVelocity().getValueAsDouble();
        double targetRPS = this.rpsFromDistanceRegression(distance);
        return Math.abs(currentRPS - targetRPS) < 5.0;
    }

    // *Stops all motors
    public void stop() {
        shooterMotor1.stopMotor();
        kicker1.stopMotor();
    }

    public void stopKicker() {
        kicker1.stopMotor();
    }

    public double getVelocity() {
        return shooterMotor1.getVelocity().getValueAsDouble();
    }
}