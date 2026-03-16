package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.VisionSimSystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.VisionConstants;

//Alternate AimAndShoot that uses PhotonUtils.getYawToPose() and PhotonVision.getDistanceToPose().
//With this, we can aim directly at the hub instead of relying on AprilTags
//However, we need to make sure our fieldToRobot updating is correct.

public class AimAndShoot2 extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;
    private VisionSimSystem vision;
    private final PIDController rotController;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(0.1)
        .withDriveRequestType(DriveRequestType.Velocity);
        
    public AimAndShoot2(
            CommandSwerveDrivetrain drivetrain,
            VisionSimSystem vision, 
            Shooter shooter, 
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {
    
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.shooter = shooter;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        rotController = new PIDController(10, .05, .005);
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain, vision, shooter);
    }

    @Override
    public void initialize() {
        rotController.setTolerance(0.00001);
        rotController.setSetpoint(0);
    }

    @Override
    public void execute() {
        double theta = -vision.getYawToPose(VisionConstants.getHubPose());
        double distance = vision.getDistanceToPose(VisionConstants.getHubPose());
        double rotSpeed = rotController.calculate(theta);

        // Driver still controls translation, command controls rotation
        drivetrain.setControl(
            drive
                .withVelocityX(xSupplier.getAsDouble())
                .withVelocityY(ySupplier.getAsDouble())
                .withRotationalRate(rotSpeed)
        );
        
        if (rotController.atSetpoint()) {
            shooter.rpmControl(distance);
            if (shooter.atCorrectRPM(distance)) {
                shooter.spinKickers();
            }
        } else {
            shooter.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
        drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
    );
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}