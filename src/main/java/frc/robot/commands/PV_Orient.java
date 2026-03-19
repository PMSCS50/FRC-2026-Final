package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class PV_Orient extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final int targetId;
    private final DoubleSupplier xInput;
    private final DoubleSupplier yInput;

    private final PIDController rotController = new PIDController(0.05, 0, 0.002);
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    public PV_Orient(
        CommandSwerveDrivetrain drivetrain,
        VisionSubsystem vision,
        int targetId,
        double rotSetpoint, // kept for compatibility but should always be 0
        DoubleSupplier xInput,
        DoubleSupplier yInput
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetId = targetId;
        this.xInput = xInput;
        this.yInput = yInput;

        rotController.setTolerance(1.0); // degrees
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        rotController.reset();
        rotController.setSetpoint(0); // we want tag yaw = 0, meaning centered
    }

    @Override
    public void execute() {
        if (!vision.hasTarget(targetId)) {
            // no target — let driver still translate, just stop rotating
            drivetrain.setControl(
                drive.withVelocityX(xInput.getAsDouble())
                     .withVelocityY(yInput.getAsDouble())
                     .withRotationalRate(0)
            );
            return;
        }

        double yaw = vision.getTargetYaw(targetId); // degrees, from PhotonVision directly
        double rotVel = rotController.calculate(yaw);

        SmartDashboard.putNumber("PV_Orient/TargetYaw", yaw);
        SmartDashboard.putNumber("PV_Orient/RotVel", rotVel);
        SmartDashboard.putBoolean("PV_Orient/AtSetpoint", rotController.atSetpoint());

        drivetrain.setControl(
            drive.withVelocityX(xInput.getAsDouble())
                 .withVelocityY(yInput.getAsDouble())
                 .withRotationalRate(rotVel)
        );
    }

    @Override
    public boolean isFinished() {
        return false; // whileTrue handles termination on button release
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}