package frc.robot.subsystems;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class    Shooter extends SubsystemBase {

    // ************************
    // MOTORS
    // ************************

    public static TalonFX shooterMotor1;
        private final TalonFX shooterMotor2;
        private final SparkMax kicker1 = new SparkMax(ShooterConstants.kickerMotorCanId1, MotorType.kBrushless);
        private final SparkMax kicker2 = new SparkMax(ShooterConstants.kickerMotorCanId2, MotorType.kBrushless);
    
        private final VisionSubsystem vision;
    
    
        // ************************
        // MOTOR CONTROLS
        // ************************
    
        // DutyCycleOut: simple 0-1 power control, used by setShooterSpeed()
        private final DutyCycleOut motorControl = new DutyCycleOut(0.0);
    
        // VelocityVoltage: closed-loop RPM control, used by setVelocityTo()
        private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);
    
        // ************************
        // SHOOTER PHYSICS CONSTANTS
        // ************************
    
        private double velocity = 0.0;
        private final double shooterAngle = 70.0;                  // degrees
        private final double phi = Math.toRadians(shooterAngle);   // radians
        private final double shooterHeight = 0.508;                // meters from ground
    
    

    // CONSTRUCTOR
    public Shooter(VisionSubsystem vision) {
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

        // Kicker config
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

    // Applies shared TalonFX settings — PID gains, current limits, neutral mode
    private void configureShooterMotor(TalonFXConfiguration config) {
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Velocity PID gains — tune these on the real robot
        config.Slot0.kS = 0;   // static friction compensation (volts)
        config.Slot0.kV = 0.1150;  // velocity feedforward
        config.Slot0.kP = .2; // .8   // proportional
        config.Slot0.kI = 0;     // integral
        config.Slot0.kD = 0;     // derivative
    }


    // ************************
    // PERIODIC
    // ************************

    @Override
    public void periodic()   
    {
        // SmartDashboard.putNumber("Shooter RPM", shooterMotor1.getVelocity().getValueAsDouble() * 60.0);

    }


    // ************************
    // SHOOTING METHODS
    // ************************

    /**
     * Simple duty cycle control (0.0 to 1.0).
     * Used for manual testing from RobotContainer buttons.
     * Also runs kickers at full power.
     */
    public void setShooterSpeed(double speed) {
        shooterMotor1.setControl(motorControl.withOutput(speed));
        kicker1.set(1);
        // SmartDashboard.putNumber("Actual RPS", shooterMotor1.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Motor Output", shooterMotor1.getMotorVoltage().getValueAsDouble());
    }

   

     public void rpmControl(double distance) {
        double rpm = vision.rpmFromDistanceRegression(distance);
        double rps = rpm / 60;
        shooterMotor1.setControl(velocityRequest.withVelocity(rps));
        SmartDashboard.putNumber("Target RPM", rpm);
        SmartDashboard.putNumber("Target RPS", rps);
        SmartDashboard.putNumber("Actual RPS", shooterMotor1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Motor Output", shooterMotor1.getMotorVoltage().getValueAsDouble());
    }

    public void rpmControlDuty(int id) {
        double rpm = vision.rpmFromDistanceRegression(vision.getDistance(id));
        double percentOutput = rpm / 2500.0;
        shooterMotor1.setControl(motorControl.withOutput(percentOutput));
        
    }
    
    public void spinKickers() {
        kicker1.set(1);
    }

    public boolean atCorrectRPM(int id) {

        double rotationsPerSecond = shooterMotor1.getVelocity().getValueAsDouble();
        double currentRPM = rotationsPerSecond * 60.0;
        double targetRPM = vision.rpmFromDistanceRegression(vision.getDistance(id));
        return Math.abs(currentRPM - targetRPM) < 500.0;
        
    }

    public boolean atCorrectRPMFixed(double distance) {
        double rotationsPerSecond = shooterMotor1.getVelocity().getValueAsDouble();
        double currentRPM = rotationsPerSecond * 60;
        double targetRPM = vision.rpmFromDistanceRegression(distance);
        return Math.abs(currentRPM - targetRPM) < 1000.0;
    }

    /** Stops all motors */
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

    /**
     * Calculates required exit velocity (m/s) to reach target at horizontal distance x (meters).
     * Assumes flat ground and fixed shooter angle (phi).
     */
    

    // /**f
    //  * Velocity calculation variant for shooting while climbing.
    //  * Accounts for robot height offset and pitch angle.
    //  */
    // public double velocityFromDistance(double x, double robotZ, double pitch) {
    //     double y = 1.8288 - shooterHeight - robotZ;

    //     // FIX: was "double phi = phi + pitch" which caused variable shadowing
    //     double adjustedPhi = this.phi + pitch;

    //     double v = Math.sqrt(
    //         (9.807 * x * x) /
    //         (2 * Math.cos(adjustedPhi) * Math.cos(adjustedPhi) * (x * Math.tan(adjustedPhi) + shooterHeight - y))
    //     );

    //     double dragFactor = (1 + 0.015 * x) * 1.04;
    //     return dragFactor * v;
    // }

    
    // public double[] correctVandYaw(double dx, double dy, double yaw, double vxField, double vyField) {
    //     double distance = Math.hypot(dx, dy);

    //     double unitX = dx / distance;
    //     double unitY = dy / distance;

    //     double vStationary = velocityFromDistance(distance);
    //     double vHorizontal = vStationary * Math.cos(phi);

    //     double correctedVx = vHorizontal * unitX;
    //     double correctedVy = vHorizontal * unitY;

    //     if (Math.hypot(vxField, vyField) > 0.01) {
    //         correctedVx -= vxField;
    //         correctedVy -= vyField;

    //         double correctedYaw = Math.atan2(correctedVy, correctedVx);
    //         double correction = MathUtil.angleModulus(correctedYaw - yaw);

    //         correction = MathUtil.clamp(correction, -0.2, 0.2);
    //         yaw += correction;
    //     }

    //     return new double[]{ correctedVx, correctedVy, yaw };
    // }
        



    /** Returns the current target velocity in m/s */
    


   
    

   

   
}