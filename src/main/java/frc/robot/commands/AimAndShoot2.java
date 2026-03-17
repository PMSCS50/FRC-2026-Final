package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.VisionSimSystem;
import frc.robot.subsystems.vision.VisionSubsystem;

//import static edu.wpi.first.units.Units.Degree;

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
    private static final double SHOOT_COOLDOWN = .05;

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
        double aimAngle = Math.PI/3;

        double linVelocity = vision.getShooterVelocity(distance) * 1.5;
        double xVelocity = linVelocity * Math.cos(aimAngle);
        double yVelocity = linVelocity * Math.sin(aimAngle);
        double robotHeading = drivetrain.getPose().getRotation().getRadians();
        double robotHeading2 = robotHeading + 1 * Math.PI/6;
        double robotHeading3 = robotHeading + 2 * Math.PI/6;
        double robotHeading4 = robotHeading + 3 * Math.PI/6;
        double robotHeading5 = robotHeading + 4 * Math.PI/6;
        double robotHeading6 = robotHeading + 5 * Math.PI/6;
        double robotHeading7 = robotHeading + 6 * Math.PI/6;
        double robotHeading8 = robotHeading + 7 * Math.PI/6;
        double robotHeading9 = robotHeading + 8 * Math.PI/6;
        double robotHeading10 = robotHeading + 9 * Math.PI/6;
        double robotHeading11 = robotHeading + 10 * Math.PI/6;
        double robotHeading12 = robotHeading + 11 * Math.PI/6;
        
        Translation3d shooterVelocity = new Translation3d(
            xVelocity * Math.cos(robotHeading),
            xVelocity * Math.sin(robotHeading),
            yVelocity
        );

        Translation3d shooterVelocity1 = new Translation3d(
            xVelocity * Math.cos(robotHeading2),
            xVelocity * Math.sin(robotHeading2),
            yVelocity
        );

        Translation3d shooterVelocity2 = new Translation3d(
            xVelocity * Math.cos(robotHeading3),
            xVelocity * Math.sin(robotHeading3),
            yVelocity
        );

        Translation3d shooterVelocity3 = new Translation3d(
            xVelocity * Math.cos(robotHeading4),
            xVelocity * Math.sin(robotHeading4),
            yVelocity
        );

        Translation3d shooterVelocity4 = new Translation3d(
            xVelocity * Math.cos(robotHeading5),
            xVelocity * Math.sin(robotHeading5),
            yVelocity
        );

        Translation3d shooterVelocity5 = new Translation3d(
            xVelocity * Math.cos(robotHeading6),
            xVelocity * Math.sin(robotHeading6),
            yVelocity
        );

        Translation3d shooterVelocity6 = new Translation3d(
            xVelocity * Math.cos(robotHeading7),
            xVelocity * Math.sin(robotHeading7),
            yVelocity
        );

        Translation3d shooterVelocity7 = new Translation3d(
            xVelocity * Math.cos(robotHeading8),
            xVelocity * Math.sin(robotHeading8),
            yVelocity
        );

        Translation3d shooterVelocity8 = new Translation3d(
            xVelocity * Math.cos(robotHeading9),
            xVelocity * Math.sin(robotHeading9),
            yVelocity
        );

        Translation3d shooterVelocity9 = new Translation3d(
            xVelocity * Math.cos(robotHeading10),
            xVelocity * Math.sin(robotHeading10),
            yVelocity
        );

        Translation3d shooterVelocity10 = new Translation3d(
            xVelocity * Math.cos(robotHeading11),
            xVelocity * Math.sin(robotHeading11),
            yVelocity
        );

        Translation3d shooterVelocity11 = new Translation3d(
            xVelocity * Math.cos(robotHeading12),
            xVelocity * Math.sin(robotHeading12),
            yVelocity
        );

        Translation3d drivePose = new Translation3d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), 0);

        // Driver still controls translation, command controls rotation
        drivetrain.setControl(
            drive
                .withVelocityX(xSupplier.getAsDouble())
                .withVelocityY(ySupplier.getAsDouble())
                .withRotationalRate(rotSpeed)
        );
        
        if (rotController.atSetpoint() && shootTimer.hasElapsed(SHOOT_COOLDOWN)) {
            ballSim.launchBall(drivePose, shooterVelocity, vision.rpmFromDistance(distance));
            ballSim.launchBall(drivePose, shooterVelocity1, vision.rpmFromDistance(distance));
            ballSim.launchBall(drivePose, shooterVelocity2, vision.rpmFromDistance(distance));
            ballSim.launchBall(drivePose, shooterVelocity3, vision.rpmFromDistance(distance));
            ballSim.launchBall(drivePose, shooterVelocity4, vision.rpmFromDistance(distance));
            ballSim.launchBall(drivePose, shooterVelocity5, vision.rpmFromDistance(distance));
            ballSim.launchBall(drivePose, shooterVelocity6, vision.rpmFromDistance(distance));
            ballSim.launchBall(drivePose, shooterVelocity7, vision.rpmFromDistance(distance));
            ballSim.launchBall(drivePose, shooterVelocity8, vision.rpmFromDistance(distance));
            ballSim.launchBall(drivePose, shooterVelocity9, vision.rpmFromDistance(distance));
            ballSim.launchBall(drivePose, shooterVelocity10, vision.rpmFromDistance(distance));
            ballSim.launchBall(drivePose, shooterVelocity11, vision.rpmFromDistance(distance));
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