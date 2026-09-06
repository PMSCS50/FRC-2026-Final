package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {

    // Shooter motors (simulated TalonFX)
    private final TalonFX shooter1 = new TalonFX(ShooterConstants.shooterMotorCanId1);
    private final TalonFX shooter2 = new TalonFX(ShooterConstants.shooterMotorCanId2);

    private final TalonFXSimState shooter1Sim = shooter1.getSimState();
    private final TalonFXSimState shooter2Sim = shooter2.getSimState();

    // Kicker motors (SparkMax sim)
    private final SparkMax kicker1 = new SparkMax(ShooterConstants.kickerMotorCanId1, MotorType.kBrushless);
    private final SparkMax kicker2 = new SparkMax(ShooterConstants.kickerMotorCanId2, MotorType.kBrushless);

    private final RelativeEncoder kicker1Encoder = kicker1.getEncoder();

    // Flywheel simulation state
    private double shooterVelocityRPS = 0.0;
    private double appliedShooterVoltage = 0.0;
    private double targetShooterRPS = 0.0;

    // Simple flywheel physics constants
    private static final double kV = 0.12;     // volts → rps gain
    private static final double damping = 0.08; // drag coefficient
    private static final double kP = 6.0;       // closed-loop velocity gain (sim only)

    public ShooterIOSim() {
        shooter1Sim.setSupplyVoltage(12.0);
        shooter2Sim.setSupplyVoltage(12.0);
    }

    @Override
    public void setShooterVoltage(double volts) {
        appliedShooterVoltage = volts;
    }

    @Override
    public void setShooterVelocity(double rps) {
        targetShooterRPS = rps;
    }

    @Override
    public void setKickerVoltage(double volts) {
        kicker1.setVoltage(volts);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {

        double dt = 0.02;

        // --- Closed-loop velocity simulation (VelocityVoltage equivalent) ---
        double closedLoopAccel = (targetShooterRPS - shooterVelocityRPS) * kP;

        // --- Open-loop voltage simulation ---
        double openLoopAccel = appliedShooterVoltage * kV;

        // --- Combined physics ---
        double accel = closedLoopAccel + openLoopAccel - (shooterVelocityRPS * damping);
        shooterVelocityRPS += accel * dt;

        // Update TalonFX sim states
        shooter1Sim.setRotorVelocity(shooterVelocityRPS);
        shooter2Sim.setRotorVelocity(shooterVelocityRPS);

        // --- Simulate kicker2 following kicker1 ---
        kicker2.set(kicker1.get());

        // --- Fill IO fields ---
        inputs.shooterVelocity = shooterVelocityRPS;
        inputs.kickerVelocity = kicker1Encoder.getVelocity();

        inputs.totalCurrent =
            shooter1Sim.getTorqueCurrent() +
            kicker1.getOutputCurrent();
    }
}
