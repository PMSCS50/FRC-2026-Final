package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
// import frc.robot.subsystems.vision.LLSubsystem;
import frc.robot.subsystems.vision.LLSubsystemMany;
import java.lang.Math;

public class AlignToHub extends Command {
    private final PIDController rotController;
    private final LLSubsystemMany llvision;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    public AlignToHub(CommandSwerveDrivetrain drivetrain, LLSubsystemMany llvision) {
        this.drivetrain = drivetrain;
        this.llvision = llvision;
        this.rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);
        rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);
        rotController.enableContinuousInput(-180, 180);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotController.reset();
    }

    @Override
    public void execute() {
        if (!llvision.hasTargets()) {
            drivetrain.setControl(drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
            );
            return;
        }

        double angleToHub = llvision.getYawToTarget(VisionConstants.getHubPose()) * 180 / Math.PI;
        double currentHeading = llvision.getRobotYawDeg();
        double rotValue = rotController.calculate(currentHeading, angleToHub);

        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(rotValue)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return rotController.atSetpoint();
    }
}           