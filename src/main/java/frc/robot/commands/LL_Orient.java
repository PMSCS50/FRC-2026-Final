package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class LL_Orient extends Command {
    private final PIDController rotController;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    private final DoubleSupplier xInput, yInput;
    private final int targetTagID;
    private final String limelightName;

    public LL_Orient(CommandSwerveDrivetrain drivetrain, String limelightName, int targetTagID, DoubleSupplier xInput, DoubleSupplier yInput) {
        rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        this.targetTagID = targetTagID;
        this.xInput = xInput;
        this.yInput = yInput;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
        rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);
    }

    @Override
    public void execute() {
        // *Search visible tags for our target
        LimelightTarget_Fiducial[] tags = LimelightHelpers.getLatestResults(limelightName).targets_Fiducials;
        
        LimelightTarget_Fiducial lockedTag = null;
        for (LimelightTarget_Fiducial tag : tags) {
            if ((int) tag.fiducialID == targetTagID) {
                lockedTag = tag;
                break;
            }
        }

        if (lockedTag == null) {
            stop();
            return;
        }

        double yaw = lockedTag.getTargetPose_RobotSpace().getRotation().getZ();

        SmartDashboard.putNumber("LL_Orient/yaw", yaw);
        SmartDashboard.putBoolean("LL_Orient/atSetpoint", rotController.atSetpoint());

        drivetrain.setControl(drive
            .withVelocityX(xInput.getAsDouble())
            .withVelocityY(yInput.getAsDouble())
            .withRotationalRate(rotController.calculate(yaw)));
    }

    @Override
    public void end(boolean interrupted) {
        stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void stop() {
        drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
}