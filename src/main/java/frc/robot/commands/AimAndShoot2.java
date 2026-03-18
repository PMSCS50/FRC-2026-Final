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

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.firecontrol.FuelPhysicsSim;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.MatBuilder;

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
    private static final double SHOOT_COOLDOWN = .25;

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
        double shooterOffset = -.229;

        double linVelocity = vision.getShooterVelocity(distance);
        double xVelocity = linVelocity * Math.cos(aimAngle);
        double yVelocity = linVelocity * Math.sin(aimAngle);
        double robotHeading = drivetrain.getPose().getRotation().getRadians();
        
        Translation3d drivePose = new Translation3d(
            drivetrain.getPose().getX() + shooterOffset * Math.cos(robotHeading),
            drivetrain.getPose().getY() + shooterOffset * Math.sin(robotHeading),
            .5
        );

        //[c,-s, 0]
        //[s, c, 0]
        //[0, 0, 1]
        Translation3d shooterVelocity = new Translation3d(
            xVelocity * Math.cos(robotHeading),
            xVelocity * Math.sin(robotHeading),
            yVelocity 
        );

        // Driver still controls translation, command controls rotation
        drivetrain.setControl(
            drive
                .withVelocityX(xSupplier.getAsDouble())
                .withVelocityY(ySupplier.getAsDouble())
                .withRotationalRate(rotSpeed)
        );
        
        if (rotController.atSetpoint() && shootTimer.hasElapsed(SHOOT_COOLDOWN)) {
            ballSim.launchBall(drivePose, shooterVelocity, vision.rpmFromDistance(distance));
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