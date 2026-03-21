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

public class PV_Orient extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final int targetId;
    private double rotSetpoint;
    
    private final PIDController rotController = new PIDController(1, 0, 0); // kp = 2.5
    private Timer settleTimer;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    public PV_Orient(
        CommandSwerveDrivetrain drivetrain,
        VisionSubsystem vision,
        int targetId,
        double rotSetpoint
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetId = targetId;
        this.rotSetpoint = rotSetpoint;

        rotController.enableContinuousInput(-180, 180);
        rotController.setTolerance(5); // degrees instead of Math.toRadians(5)

        addRequirements(drivetrain, vision);
       
        

    }

    @Override
    public void initialize() {
        rotController.reset();
        this.settleTimer = new Timer();
    }

    @Override
    public void execute() {

        SmartDashboard.putNumber("Vision Yaw", vision.getYawFromHub(targetId));
        SmartDashboard.putBoolean("rotSetpoint", rotController.atSetpoint());
        SmartDashboard.putNumber("rot Difference", vision.getYawFromHub(targetId) - rotController.getSetpoint());


        SmartDashboard.putNumber("settle timer", settleTimer.get());


        if (!vision.hasTarget(targetId)) {
            drivetrain.setControl(new SwerveRequest.Idle());
            return;
        }

        double rotVel = 0;

      
        if (!rotController.atSetpoint()) {
            rotVel = rotController.calculate(vision.getTargetYaw(targetId), rotSetpoint);
        } else {
            rotVel = 0;
           
        }
         

        drivetrain.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(rotVel)
        );
    }

    @Override
    public boolean isFinished() {
        return rotController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}
