package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {
    
    public final TalonFX shooter1 = new TalonFX(ShooterConstants.shooterMotorCanId1);
    public final TalonFX shooter2 = new TalonFX(ShooterConstants.shooterMotorCanId2);
    private final SparkMax kicker1 = new SparkMax(ShooterConstants.kickerMotorCanId1, MotorType.kBrushless);
    private final SparkMax kicker2 = new SparkMax(ShooterConstants.kickerMotorCanId2, MotorType.kBrushless);

    private final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    private final SparkMaxConfig kickerConfig1 = new SparkMaxConfig();
    private final SparkMaxConfig kickerConfig2 = new SparkMaxConfig();

    private final RelativeEncoder kicker1Encoder = kicker1.getEncoder();
    private final RelativeEncoder kicker2Encoder = kicker2.getEncoder();
    
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withSlot(0);

    public ShooterIOReal() {
        // *Config Talons
        configureShooterMotor(shooterConfig);

        shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooter1.getConfigurator().apply(shooterConfig);

        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooter2.getConfigurator().apply(shooterConfig);
        shooter2.setControl(new Follower(shooter1.getDeviceID(), MotorAlignmentValue.Opposed));

        // *Config NEOs
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

    // *Configure the TalonFX for the shooter motors
    private void configureShooterMotor(TalonFXConfiguration config) {
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Audio.AllowMusicDurDisable = true;

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

    @Override
    public void setShooterVoltage(double volts) {
        shooter1.setVoltage(volts);
    }

    @Override
    public void setKickerVoltage(double volts) {
        kicker1.setVoltage(volts);
    }

    @Override
    public void setShooterVelocity(double rps) {
        shooter1.setControl(velocityVoltage.withVelocity(rps));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterVelocity = shooter1.getVelocity().getValueAsDouble();
        inputs.kickerVelocity = kicker1Encoder.getVelocity();
        inputs.totalCurrent = shooter1.getSupplyCurrent().getValueAsDouble() + kicker1.getOutputCurrent();
    }
}
