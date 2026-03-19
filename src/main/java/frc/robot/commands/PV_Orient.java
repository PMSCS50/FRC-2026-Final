package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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

    // target yaw = 0 means tag is centered in camera, just drive that error to 0
    private final PIDController rotController = new PIDController(0.05, 0, 0.001);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(0.1)
        .withDriveRequestType(DriveRequestType.Velocity);

    public PV_Orient(
        CommandSwerveDrivetrain drivetrain,
        VisionSubsystem vision,
        int targetId,
        double rotSetpoint,
        DoubleSupplier xInput,
        DoubleSupplier yInput
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetId = targetId;
        this.xInput = xInput;
        this.yInput = yInput;

        rotController.setTolerance(1.0); // 1 degree tolerance
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        rotController.reset();
    }

    @Override
    public void execute() {
        if (!vision.hasTarget(targetId)) {
            // tag lost — hold position, let driver translate freely
            drivetrain.setControl(
                drive.withVelocityX(xInput.getAsDouble())
                     .withVelocityY(yInput.getAsDouble())
                     .withRotationalRate(0)
            );
            return;
        }

        // tag yaw from camera: 0 = centered, positive = tag is to the left, negative = right
        double tagYaw = vision.getTargetYaw(targetId);

        // drive yaw error to 0 — no odometry involved at all
        double rotVel = rotController.calculate(tagYaw, 0);

        SmartDashboard.putNumber("PV_Orient/TagYaw", tagYaw);
        SmartDashboard.putNumber("PV_Orient/RotVel", rotVel);
        SmartDashboard.putBoolean("PV_Orient/Centered", rotController.atSetpoint());

        drivetrain.setControl(
            drive.withVelocityX(xInput.getAsDouble())
                 .withVelocityY(yInput.getAsDouble())
                 .withRotationalRate(rotVel)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}