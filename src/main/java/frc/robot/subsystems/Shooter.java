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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    // ************************
    // MOTORS
    // ************************

    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final SparkMax kicker1 = new SparkMax(ShooterConstants.kickerMotorCanId1, MotorType.kBrushless);
    private final SparkMax kicker2 = new SparkMax(ShooterConstants.kickerMotorCanId2, MotorType.kBrushless);

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


    // ************************
    // CONSTRUCTOR
    // ************************

    public Shooter() {
        shooterMotor1 = new TalonFX(ShooterConstants.shooterMotorCanId1);
        shooterMotor2 = new TalonFX(ShooterConstants.shooterMotorCanId2);

        // Build and apply shared config
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        configureShooterMotor(shooterConfig);

        // Motor 1 specific: set direction
        shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterMotor1.getConfigurator().apply(shooterConfig);

        // Motor 2: reset invert to default before applying, then follow motor 1
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
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40);

        kicker1.configure(kickerConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kicker2.configure(kickerConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Applies shared TalonFX settings — PID gains, current limits, neutral mode
    private void configureShooterMotor(TalonFXConfiguration config) {
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Velocity PID gains — tune these on the real robot
        config.Slot0.kS = 0.1;   // static friction compensation (volts)
        config.Slot0.kV = 0.12;  // velocity feedforward
        config.Slot0.kP = 0.11;  // proportional
        config.Slot0.kI = 0;     // integral
        config.Slot0.kD = 0;     // derivative
    }


    // ************************
    // PERIODIC
    // ************************

    @Override
    public void periodic() {}


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
        //kicker1.set(.2);
        //kicker2.set(-.2);
    }

    /**
     * Closed-loop velocity control in m/s.
     * Converts to RPM internally and uses PID to hold that speed.
     * Use this for autonomous and distance-based shooting.
     */
    public void setVelocityTo(double newVelocity) {
        velocity = newVelocity;
        // VelocityVoltage takes rotations per second, so divide RPM by 60
        shooterMotor1.setControl(velocityRequest.withVelocity(convertToRPM(velocity) / 60.0));
    }

    /** Runs kickers at the configured power from ShooterConstants */
    public void startKickerMotors() {
        kicker1.set(ShooterConstants.kickerMotorPower);
        kicker2.set(-ShooterConstants.kickerMotorPower);
    }

    /** Stops kicker motors */
    public void stopKickerMotors() {
        kicker1.set(0);
        kicker2.set(0);
    }

    /** Stops all motors */
    public void stop() {
        shooterMotor1.stopMotor();
        kicker1.stopMotor();
        kicker2.stopMotor();
    }

    /**
     * Calculates required exit velocity (m/s) to reach target at horizontal distance x (meters).
     * Assumes flat ground and fixed shooter angle (phi).
     */
    public double velocityFromDistance(double x) {
        double y = 1.8288 - shooterHeight; // target height minus shooter height

        double v = Math.sqrt(
            (9.807 * x * x) /
            (2 * Math.cos(phi) * Math.cos(phi) * (x * Math.tan(phi) + shooterHeight - y))
        );

        // Empirical drag correction — increases with distance
        double dragFactor = (1 + 0.015 * x) * 1.04;
        return dragFactor * v;
    }

    /**
     * Velocity calculation variant for shooting while climbing.
     * Accounts for robot height offset and pitch angle.
     */
    public double velocityFromDistance(double x, double robotZ, double pitch) {
        double y = 1.8288 - shooterHeight - robotZ;

        // FIX: was "double phi = phi + pitch" which caused variable shadowing
        double adjustedPhi = this.phi + pitch;

        double v = Math.sqrt(
            (9.807 * x * x) /
            (2 * Math.cos(adjustedPhi) * Math.cos(adjustedPhi) * (x * Math.tan(adjustedPhi) + shooterHeight - y))
        );

        double dragFactor = (1 + 0.015 * x) * 1.04;
        return dragFactor * v;
    }

    /* 
    public double[] correctVandYaw(double dx, double dy, double yaw, double vxField, double vyField) {
        double distance = Math.hypot(dx, dy);

        double unitX = dx / distance;
        double unitY = dy / distance;

        double vStationary = velocityFromDistance(distance);
        double vHorizontal = vStationary * Math.cos(phi);

        double correctedVx = vHorizontal * unitX;
        double correctedVy = vHorizontal * unitY;

        if (Math.hypot(vxField, vyField) > 0.01) {
            correctedVx -= vxField;
            correctedVy -= vyField;

            double correctedYaw = Math.atan2(correctedVy, correctedVx);
            double correction = MathUtil.angleModulus(correctedYaw - yaw);

            correction = MathUtil.clamp(correction, -0.2, 0.2);
            yaw += correction;
        }

        return new double[]{ correctedVx, correctedVy, yaw };
    }
        */



    /** Returns the current target velocity in m/s */
    public double getVelocity() {
        return velocity;
    }

    /** Returns true if the shooter is currently running */
    public boolean isShooting() {
        return Math.abs(velocity) > 0.01;
    }

   
    public boolean atCorrectRPM() {
        // Phoenix 6: getVelocity() returns rotations per second
        double rotationsPerSecond = shooterMotor1.getVelocity().getValueAsDouble();
        double currentRPM = rotationsPerSecond * 60.0;
        double targetRPM = convertToRPM(velocity);

        // Use tolerance instead of exact equality
        return Math.abs(currentRPM - targetRPM) < 50.0;
    }

   
    private double convertToRPM(double velocity) {
        double wheelRadius = 0.0508; // meters (~2 inches)
        double c = 1.0;             // tuning constant, adjust if needed
        return c * (velocity * 60.0) / (2.0 * Math.PI * wheelRadius);
    }
}