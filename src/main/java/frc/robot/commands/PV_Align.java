package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
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
    
    private PIDController xController = new PIDController(1.45, 0.001, 0); // kp = 1.45, ki = .001
    private final PIDController yController = new PIDController(1, 0, 0); // kp = 1
    private final PIDController rotController = new PIDController(2.5, 0, 0); // kp = 2.5
    private Timer settleTimer;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    private double SETTLETIME = .1;

    public PV_Align(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetId = vision.getTargetId();

        rotController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        rotController.setTolerance(Math.toRadians(5));

        addRequirements(drivetrain, vision);
    }

    public PV_Align(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, int targetId) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetId = targetId;

        rotController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(.05);
        yController.setTolerance(.05);
        rotController.setTolerance(Math.toRadians(5));

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        rotController.reset();
        this.settleTimer = new Timer();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Vision X", vision.getX());
        SmartDashboard.putNumber("Vision Y", vision.getY());
        SmartDashboard.putNumber("Vision Yaw", vision.getYawRad());

        SmartDashboard.putBoolean("rotSetpoint", rotController.atSetpoint());
        SmartDashboard.putNumber("rot Difference", vision.getYawRad() - rotController.getSetpoint());

        SmartDashboard.putNumber("x Difference", vision.getX() - xController.getSetpoint());
        SmartDashboard.putBoolean("x Setpoint", xController.atSetpoint());

        SmartDashboard.putNumber("y Difference", vision.getY() - yController.getSetpoint());
        SmartDashboard.putBoolean("y Setpoint", yController.atSetpoint());

        SmartDashboard.putNumber("settle timer", settleTimer.get());

        if (!vision.hasTarget(targetId)) {
            drivetrain.setControl(new SwerveRequest.Idle());
            return;
        }

        
        // double xVel = MathUtil.clamp(xController.calculate(
        //     vision.getX(), DESIRED_DISTANCE_METERS
        // ),-2,0.05);

        // double yVel = yController.calculate(
        //     vision.getY(), 0
        // );

        // double rotVel = rotController.calculate(
        //     vision.getYawRad(), 0
        // );
        
        double xVel = 0;
        double yVel = 0;
        double rotVel = 0;

        if (!xController.atSetpoint()) {
            xVel = xController.calculate(vision.getX(), DESIRED_DISTANCE_METERS);
        } else {
            xVel = 0;
        }
        // if (!yController.atSetpoint()) {
        //     yVel = yController.calculate(vision.getY(), 0);
        //     settleTimer.stop();
        //     settleTimer.reset();
        // } else {
        //     settleTimer.start();
        //     if (this.settleTimer.hasElapsed(SETTLETIME)) {
        //         yVel = 0;
            
        //     } else {
        //         yVel = yController.calculate(vision.getY(), 0);
        //     }
        // }
        if (!yController.atSetpoint()) {
            yVel = yController.calculate(vision.getY(), 0);
        } else {
            yVel = 0;
        }
        if (!rotController.atSetpoint()) {
            rotVel = rotController.calculate(vision.getYawRad(), 0);
        } else {
            rotVel = 0;
           
        }
        // if (!yController.atSetpoint()) yVel = yController.calculate(vision.getY(), 0);
         

        drivetrain.setControl(
            drive.withVelocityX(-xVel)
                 .withVelocityY(-yVel)
                 .withRotationalRate(rotVel)
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
