package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.firecontrol.FuelPhysicsSim;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.VisionSimSystem;

public class AutoShoot extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;
    private VisionSimSystem vision;
    private FuelPhysicsSim ballSim;

    private final PIDController rotController;
    private final Timer shootTimer = new Timer();
    private final Timer endTimer = new Timer();
    private static final double SHOOT_COOLDOWN = .25;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(0.1)
        .withDriveRequestType(DriveRequestType.Velocity);

    public AutoShoot(CommandSwerveDrivetrain drivetrain, Shooter shooter, VisionSimSystem vision, FuelPhysicsSim ballSim) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.shooter = shooter;
        this.ballSim = ballSim;

        rotController = new PIDController(10, .05, .005);
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain, vision, shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rotController.setTolerance(2);
        rotController.setSetpoint(0);
        shootTimer.restart();
        endTimer.restart();
        //RobotContainer.ballSim.addBall(RobotContainer.drivetrain.getPose().plus(new Translation3d(0, 0, 1.5)), new Translation3d(0, 0, 10));
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

        Translation3d shooterVelocity = new Translation3d(
            xVelocity * Math.cos(robotHeading),
            xVelocity * Math.sin(robotHeading),
            yVelocity 
        );

        drivetrain.setControl(drive.withRotationalRate(rotSpeed));

        if (rotController.atSetpoint() && shootTimer.hasElapsed(SHOOT_COOLDOWN)) {
            ballSim.launchBall(drivePose, shooterVelocity, vision.rpmFromDistance(distance));
            shootTimer.restart();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return endTimer.hasElapsed(4);
    }
    
}
