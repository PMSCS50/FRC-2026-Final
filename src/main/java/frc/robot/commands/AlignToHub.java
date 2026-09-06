package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;

public class AlignToHub extends Command {

    private final PIDController rotController;
    private final Vision vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    // Hysteresis state: remembers which direction we were turning
    // so we don't dither when the error is near the +/-180 boundary.
    private double lastYawErrorDeg = 0.0;

    // How close to the +/-180 boundary counts as "antipodal" and
    // needs direction locking instead of trusting the raw wrapped error.
    private static final double ANTIPODAL_DEADBAND_DEG = 2.0;

    public AlignToHub(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        rotController = new PIDController(3, .5, .05);
        rotController.enableContinuousInput(-180, 180);
        rotController.setTolerance(.01);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotController.reset();
        lastYawErrorDeg = 0.0;
    }

    @Override
        public void execute() {
            Pose2d hubPose = vision.getCachedHubPose();

            if (hubPose == null) {
                drivetrain.setControl(drive.withRotationalRate(0));
                return;
            }

            double rawYawErrorDeg = vision.getYawToPose(hubPose);
            double yawErrorDeg = rawYawErrorDeg;

            if (Math.abs(rawYawErrorDeg) > (180.0 - ANTIPODAL_DEADBAND_DEG)) {
                double lockedSign = (lastYawErrorDeg != 0.0)
                    ? Math.signum(lastYawErrorDeg)
                    : 1.0;
                yawErrorDeg = lockedSign * Math.abs(rawYawErrorDeg);
            }

            lastYawErrorDeg = yawErrorDeg;

            double rotCmdDegPerSec = -rotController.calculate(yawErrorDeg, 0);
            double rotCmdRadPerSec = Math.toRadians(rotCmdDegPerSec);
            
            drivetrain.setControl(
                drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(rotCmdRadPerSec)
            );
        }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive.withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return rotController.atSetpoint();
    }
}