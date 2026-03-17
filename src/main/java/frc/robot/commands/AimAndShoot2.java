package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.VisionSimSystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import static edu.wpi.first.units.Units.Degree;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.firecontrol.FuelPhysicsSim;
import edu.wpi.first.wpilibj.Timer;

// Aims to the hub and shoots with a velocity based on the distance to the hub. Aiming works in sim, shooting has not been tested.
public class AimAndShoot2 extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;
    private VisionSimSystem vision;
    private final PIDController rotController;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private FuelPhysicsSim ballSim;

    private final Timer shootTimer = new Timer();
    private static final double SHOOT_COOLDOWN = 0.00000001;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(0.1)
        .withDriveRequestType(DriveRequestType.Velocity);
        
    public AimAndShoot2(
            CommandSwerveDrivetrain drivetrain,
            VisionSimSystem vision, 
            Shooter shooter, 
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            FuelPhysicsSim ballSim) {
    
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.shooter = shooter;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.ballSim = ballSim;

        rotController = new PIDController(10, .05, .005);
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain, vision, shooter);
    }

    @Override
    public void initialize() {
        rotController.setTolerance(2);
        rotController.setSetpoint(0);
        shootTimer.restart();
    }

    @Override
    public void execute() {
        double theta = -vision.getYawToPose(VisionConstants.getHubPose());
        double distance = vision.getDistanceToPose(VisionConstants.getHubPose());
        double rotSpeed = rotController.calculate(theta);

        double linVelocity = vision.getShooterVelocity(distance) * 1.5;
        double xVelocity = linVelocity * Math.cos(1.39626);
        double yVelocity = linVelocity * Math.sin(1.39626);
        Rotation2d robotHeading = drivetrain.getPose().getRotation();
        
        Translation3d shooterVelocity = new Translation3d(
            xVelocity * Math.cos(robotHeading.getRadians()),
            xVelocity * Math.sin(robotHeading.getRadians()),
            yVelocity
        );

        Translation3d drivePose = new Translation3d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), 0);
        Translation3d drivePose1 = new Translation3d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), 2*Math.PI/3);
        Translation3d drivePose2 = new Translation3d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), 4*Math.PI/3);

        // Driver still controls translation, command controls rotation
        drivetrain.setControl(
            drive
                .withVelocityX(xSupplier.getAsDouble())
                .withVelocityY(ySupplier.getAsDouble())
                .withRotationalRate(rotSpeed)
        );
        
        if (rotController.atSetpoint() && shootTimer.hasElapsed(SHOOT_COOLDOWN)) {
            ballSim.launchBall(drivePose, shooterVelocity, vision.rpmFromDistance(distance));
            ballSim.launchBall(drivePose1, shooterVelocity, vision.rpmFromDistance(distance));
            ballSim.launchBall(drivePose2, shooterVelocity, vision.rpmFromDistance(distance));
            shootTimer.restart();
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