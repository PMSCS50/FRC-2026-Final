package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * Swerve IO inputs struct for AdvantageKit.
 * This matches CommandSwerveDrivetrain.updateInputs exactly.
 */
@AutoLog
public class SwerveIOInputs {
    /** Full module states from CTRE swerve state. */
    public SwerveModuleState[] moduleStates;

    /** Timestamp in seconds (Utils.getCurrentTimeSeconds()). */
    public double timestamp;

    /** Robot chassis speeds (vx, vy, omega). */
    public ChassisSpeeds robotChassisSpeeds;

    /** Robot heading in radians (from Pose rotation). */
    public double robotHeading;

    /** Total current draw from all drive + steer motors. */
    public double totalCurrent;

    /** Battery voltage. */
    public double totalVoltage;

    /** Whether the drive is field oriented (currently always false). */
    public boolean isFieldOriented;

    /** Full robot pose from CTRE swerve state. */
    public Pose2d robotPose;
}
