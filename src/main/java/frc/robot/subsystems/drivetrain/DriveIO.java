package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface DriveIO {

    @AutoLog
    public static class DriveIOInputs {
        public double timestamp = 0.0;

        public SwerveModuleState[] moduleStates = {
            new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState()
        };

        public double robotHeading = 0.0; // [rad]
        public ChassisSpeeds robotChassisSpeeds = new ChassisSpeeds(); // [m/s], robot-relative
        public Pose2d robotPose = new Pose2d(); // [m], field-relative

        public double totalCurrent = 0.0;
        public double totalVoltage = 0.0;

        public boolean isFieldOriented = false;
        public double distanceToHub = 0.0; // [m]

    }

    public default void updateInputs(DriveIOInputs inputs) {}
}
