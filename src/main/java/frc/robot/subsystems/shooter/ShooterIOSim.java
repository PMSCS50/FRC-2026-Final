package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import frc.robot.util.misc.VirtualPD;

public class ShooterIOSim implements ShooterIO {

    private static final double kShooterMOI = 0.000813538;
    private static final double kShooterGearing = 1.0;
    private static final double kKickerMOI = 0.0001; 

    private boolean velocityControl = false;
    private double targetShooterRPS = 0.0;
    private double shooterVolts = 0.0;
    private double kickerVolts = 0.0;
    private double shooterAngle = 0.0;

    private final PIDController shooterPID =
        new PIDController(0.2, 0.0, 0.0);

    // Initial feedforward estimate; tune against the simulated plant.
    private final SimpleMotorFeedforward shooterFF =
        new SimpleMotorFeedforward(0.0, 0.115, 0.0);

    private final DCMotor gearbox = DCMotor.getKrakenX60(2);

    private final FlywheelSim shooterSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            gearbox,
            kShooterMOI,
            kShooterGearing
        ),
        gearbox
    );

    private final FlywheelSim kickerSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getNeo550(2),
            kKickerMOI,
            1.0
        ),
        DCMotor.getNeo550(2)
    );

    public final LoggedMechanism2d shooterVisual = new LoggedMechanism2d(3, 3);
    private final LoggedMechanismRoot2d shooterRoot = shooterVisual.getRoot("Shooter", 1.5, 1);
    private final LoggedMechanismLigament2d shooterLigament = new LoggedMechanismLigament2d("shooterWheel", 0.8, 0.0);

    public ShooterIOSim() {
        shooterRoot.append(shooterLigament);
        registerMotors();
    }

    public void registerMotors() {
        VirtualPD.registerMotor(() -> Amps.of(shooterSim.getCurrentDrawAmps()), "Shooter");
        VirtualPD.registerMotor(() -> Amps.of(kickerSim.getCurrentDrawAmps()), "Kicker");
    }

    @Override
    public void setShooterVoltage(double volts) {
        velocityControl = false;
        shooterVolts = volts;
    }

    @Override
    public void setShooterVelocity(double rps) {
        velocityControl = true;
        targetShooterRPS = rps;
    }

    @Override
    public void setKickerVoltage(double volts) {
        kickerVolts = volts;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Shooter velocity control
        if (velocityControl) {
            double targetRadPerSec = targetShooterRPS * 2.0 * Math.PI;
            double currentRadPerSec =
                shooterSim.getAngularVelocityRadPerSec();

            double pidVolts = shooterPID.calculate(
                currentRadPerSec,
                targetRadPerSec
            );

            double ffVolts = shooterFF.calculate(targetRadPerSec);

            shooterVolts = MathUtil.clamp(
                pidVolts + ffVolts,
                -RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage()
            );

            Logger.recordOutput("Shooter/Sim/PIDVolts", pidVolts);
            Logger.recordOutput("Shooter/Sim/FFVolts", ffVolts);
        }

        // Apply voltages and advance both simulated flywheels
        double batteryVoltage = RobotController.getBatteryVoltage();

        shooterSim.setInputVoltage(
            MathUtil.clamp(shooterVolts, -batteryVoltage, batteryVoltage)
        );

        kickerSim.setInputVoltage(
            MathUtil.clamp(kickerVolts, -batteryVoltage, batteryVoltage)
        );

        shooterSim.update(0.02);
        kickerSim.update(0.02);

        // Convert shooter angular velocity to rotations per second
        inputs.shooterVelocity =
            shooterSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

        // Preserve the existing kicker velocity convention.
        inputs.kickerVelocity =
            kickerSim.getAngularVelocityRPM() / 60.0;

        shooterLigament.setAngle(shooterAngle -= (Math.toDegrees(shooterSim.getAngularVelocityRadPerSec()) * 0.001));

        Logger.recordOutput("Shooter/Visual", shooterVisual);

    }
}