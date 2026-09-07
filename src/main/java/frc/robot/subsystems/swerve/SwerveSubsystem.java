package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.function.Supplier;

public class SwerveSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

    public SwerveSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        drivetrain.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);
    }

    public Command applyRequest(Supplier<SwerveRequest> supplier) {
        return drivetrain.applyRequest(supplier);
    }

    public SwerveDriveState getState() {
        return drivetrain.getState();
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        drivetrain.addVisionMeasurement(pose, timestamp);
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }
}
