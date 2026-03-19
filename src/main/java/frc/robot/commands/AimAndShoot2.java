package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.DriverStation;

//AimAndShoot without the need of a target id.

public class AimAndShoot2 extends Command {

    private final Shooter shooter;
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController rotController;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    public AimAndShoot2(CommandSwerveDrivetrain drivetrain, Shooter shooter, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;
        this.drivetrain = drivetrain;
        rotController = new PIDController(1, 0, 0);
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain, shooter);
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

        drivetrain.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(rotSpeed)
        );

        SmartDashboard.putNumber("Distance to hub", distance);
        SmartDashboard.putNumber("Shooter velocity", shooter.getVelocity());
        SmartDashboard.putNumber("Rotation error", theta);

        if (rotController.atSetpoint()) {

            shooter.rpmControl(distance);
            if (shooter.atCorrectRPMFixed(distance)) {
                shooter.spinKickers();
            }
            
        } else {
            shooter.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.stopKicker();
        drivetrain.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}