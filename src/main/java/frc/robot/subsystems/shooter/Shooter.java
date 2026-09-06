package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase {

    // !MOTORS
    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();

    // *VelocityVoltage: closed-loop RPM control, used by setVelocityTo()

    // !CONSTRUCTOR
    public Shooter(ShooterIO io) {
        this.io = io;
    }

    // !PERIODIC
    @Override
    public void periodic() {
        // BaseStatusSignal.refreshAll(
        //     shooterMotorVelocity,
        //     sm1SupplyCurrent, 
        //     sm2SupplyCurrent
        // );
    }

    // *Regression model by Kevin
    public double rpsFromDistanceRegression(double distance) {
        distance -= Units.inchesToMeters(24);
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
        io.setShooterVelocity(rps);
        
        // SmartDashboard.putNumber("Target RPM", rpm);
        Logger.recordOutput("Shooter/rpsControl/Target RPS", rps);
        Logger.recordOutput("Shooter/rpsControl/Actual RPS", inputs.shooterVelocity);
        Logger.recordOutput("Shooter/rpsControl/RPS Difference", rps - inputs.shooterVelocity);
    }

    // *Runs the kicker motors at full power to feed balls into the shooter.
    public void spinKickersMax() {
        io.setKickerVoltage(12.0);
    }

    // *Runs the kicker motors at a specified speed.
    public void spinKickersSpecified(double speed) {
        io.setKickerVoltage(speed);
    }

    // *Checks if the shooter is within a certain RPM threshold of the target RPM based on current distance to target.
    public boolean atCorrectRPS(double distance) {
        double currentRPS = inputs.shooterVelocity;
        double targetRPS = this.rpsFromDistanceRegression(distance);
        boolean atCorrectRPS = Math.abs(currentRPS - targetRPS) < 5.0;
        Logger.recordOutput("Shooter/rpsControl/atCorrectRPS", atCorrectRPS);
        return atCorrectRPS;
    }

    // *Same as atCorrectRPM but with a tighter threshold for more precise shooting.
    // ?Useful for testing and tuning the regression model and PID gains.
    public boolean atCorrectRPSFixed(double distance) {
        double currentRPS = inputs.shooterVelocity;
        double targetRPS = this.rpsFromDistanceRegression(distance);
        return Math.abs(currentRPS - targetRPS) < 5.0;
    }

    // *Stops all motors
    public void stop() {
        io.setShooterVoltage(0.0);
        io.setKickerVoltage(0.0);
    }

    public void stopKicker() {
        io.setKickerVoltage(0.0);
    }

    // *Getters
    public double getShooterVelocity() {
        return inputs.shooterVelocity;
    }

    public double getKickerVelocity() {
        return inputs.kickerVelocity;
    }

    public TalonFX getShooterMotor1() {
        if (io instanceof ShooterIOReal real) {
            return real.shooter1;
        }
        return null;
    }

    public TalonFX getShooterMotor2() {
        if (io instanceof ShooterIOReal real) {
            return real.shooter2;
        }
        return null;
    }

}