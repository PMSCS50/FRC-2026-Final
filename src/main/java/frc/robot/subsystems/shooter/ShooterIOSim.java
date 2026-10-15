package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import frc.robot.Constants.ShooterConstants;
import frc.robot.util.misc.VirtualPD;

public class ShooterIOSim implements ShooterIO {

    private final TalonFX shooter1 =
        new TalonFX(ShooterConstants.shooterMotorCanId1);
    private final TalonFX shooter2 =
        new TalonFX(ShooterConstants.shooterMotorCanId2);

    private final TalonFXSimState shooter1Sim = shooter1.getSimState();
    private final TalonFXSimState shooter2Sim = shooter2.getSimState();

    private final SparkMax kicker1 =
        new SparkMax(ShooterConstants.kickerMotorCanId1, MotorType.kBrushless);
    private final SparkMax kicker2 =
        new SparkMax(ShooterConstants.kickerMotorCanId2, MotorType.kBrushless);

    private final RelativeEncoder kicker1Encoder = kicker1.getEncoder();

    private static final double kDt = 0.02;
    private static final double kGearRatio = 1.0;
    private static final double kMomentOfInertia = 0.000813538;

    private final DCMotor shooterGearbox = DCMotor.getKrakenX60(2);

    private final FlywheelSim shooterSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            shooterGearbox,
            kMomentOfInertia,
            kGearRatio
        ),
        shooterGearbox
    );

    private final PIDController shooterPID =
        new PIDController(6.0, 0.0, 0.0);

    // Initial estimates only; tune against the simulated plant.
    // Units: volts, volts/(rad/s), and volts/(rad/s^2).
    private final SimpleMotorFeedforward shooterFF =
        new SimpleMotorFeedforward(0.0, 0.12, 0.0);

    private boolean pidControl = false;
    private double targetShooterRPS = 0.0;
    private double appliedShooterVoltage = 0.0;

    public ShooterIOSim() {
        shooter1Sim.setSupplyVoltage(12.0);
        shooter2Sim.setSupplyVoltage(12.0);
    }

    @Override
    public void registerMotors() {
        VirtualPD.registerMotor(shooter1.getStatorCurrent().asSupplier(),"Shooter");
        VirtualPD.registerMotor(() -> Amps.of(kicker1.getOutputCurrent()),"Shooter");
    }

    @Override
    public void setShooterVoltage(double volts) {
        pidControl = false;
        appliedShooterVoltage = volts;
        shooterSim.setInputVoltage(volts);
    }

    @Override
    public void setShooterVelocity(double rps) {
        pidControl = true;
        targetShooterRPS = rps;
    }

    @Override
    public void setKickerVoltage(double volts) {
        kicker1.setVoltage(volts);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        double targetRadPerSec = targetShooterRPS * 2.0 * Math.PI;
        double currentRadPerSec = shooterSim.getAngularVelocityRadPerSec();

        if (pidControl) {
            double pidVolts = shooterPID.calculate(
                currentRadPerSec,
                targetRadPerSec
            );

            double ffVolts = shooterFF.calculate(targetRadPerSec);

            appliedShooterVoltage = MathUtil.clamp(
                pidVolts + ffVolts,
                -12.0,
                12.0
            );

            Logger.recordOutput("Shooter/Sim/PIDVolts", pidVolts);
            Logger.recordOutput("Shooter/Sim/FFVolts", ffVolts);
            Logger.recordOutput(
                "Shooter/Sim/TotalVolts",
                appliedShooterVoltage
            );
            Logger.recordOutput(
                "Shooter/Sim/TargetRPS",
                targetShooterRPS
            );
        }

        shooterSim.setInputVoltage(appliedShooterVoltage);
        shooterSim.update(kDt);

        double shooterRPS =
            shooterSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

        // TalonFX rotor velocity is expressed in rotations per second.
        // This assumes a 1:1 motor-to-flywheel ratio.
        shooter1Sim.setRotorVelocity(shooterRPS);
        shooter2Sim.setRotorVelocity(shooterRPS);

        kicker2.set(kicker1.get());

        inputs.shooterVelocity = shooterRPS;
        inputs.kickerVelocity = kicker1Encoder.getVelocity();

        Logger.recordOutput("Shooter/Sim/VelocityRPS", shooterRPS);
    }
}