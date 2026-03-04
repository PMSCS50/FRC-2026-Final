package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class PV_Align extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final int targetId;

    private static final double DESIRED_DISTANCE_METERS = 1.5;
    
    private PIDController xController = new PIDController(1.5, 0, .05);
    private final PIDController yController = new PIDController(0, 0, 0.0);
    private final PIDController rotController = new PIDController(0, 0, 0);

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    public PV_Align(
        CommandSwerveDrivetrain drivetrain,
        VisionSubsystem vision,
        int targetId
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetId = targetId;

        rotController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        rotController.setTolerance(Math.toRadians(5));

        addRequirements(drivetrain, vision);
       
        

    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        rotController.reset();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Vision X", vision.getX());
        SmartDashboard.putNumber("Vision Y", vision.getY());
        SmartDashboard.putNumber("Vision Yaw", vision.getYawRad());

        if (!vision.hasTarget(targetId)) {
            drivetrain.setControl(new SwerveRequest.Idle());
            return;
        }

        double xVel = xController.calculate(
            vision.getX(), DESIRED_DISTANCE_METERS
        );

        double yVel = yController.calculate(
            vision.getY(), 0
        );

        double rotVel = rotController.calculate(
            vision.getYawRad(), 0
        );

        drivetrain.setControl(
            drive.withVelocityX(-xVel)
                 .withVelocityY(-yVel)
                 .withRotationalRate(-rotVel)
        );
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint()
            && yController.atSetpoint()
            && rotController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}
