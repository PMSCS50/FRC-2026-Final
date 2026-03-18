package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.VisionSimSystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.Constants.VisionConstants;


public class AimToPose extends Command{
    private CommandSwerveDrivetrain drivetrain;
    private VisionSimSystem vision;
    private Pose2d targetPose;
    private final PIDController rotController;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final AprilTagFieldLayout aprilTagLayout = VisionConstants.aprilTagLayout;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(0.1)
        .withDriveRequestType(DriveRequestType.Velocity);

    public AimToPose(
            CommandSwerveDrivetrain drivetrain,
            VisionSimSystem vision, 
            Pose2d targetPose, 
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {
    
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetPose = targetPose;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        rotController = new PIDController(10, .05, .005);
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain, vision);
    }

    public AimToPose(
            CommandSwerveDrivetrain drivetrain,
            VisionSimSystem vision, 
            int tagId, 
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {
    
        this.drivetrain = drivetrain;
        this.vision = vision;
        //this.targetPose = aprilTagLayout.getTagPose(tagId).toPose3d();
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        rotController = new PIDController(10, .05, .005);
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        rotController.setTolerance(0.00001);
        rotController.setSetpoint(0);
    }

    @Override
    public void execute() {
        double theta = -vision.getYawToPose(targetPose);
        //double distance = vision.getDistanceToPose(VisionConstants.getHubPose());
        double rotSpeed = rotController.calculate(theta);

        // Driver still controls translation, command controls rotation
        drivetrain.setControl(
            drive
                .withVelocityX(xSupplier.getAsDouble())
                .withVelocityY(ySupplier.getAsDouble())
                .withRotationalRate(rotSpeed)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
        drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
    
}
