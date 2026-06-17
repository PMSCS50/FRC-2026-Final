package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
// import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionGeneral;

public class PV_Align extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionGeneral vision;
    private final int targetId;

    private double xSetpoint, ySetpoint, rotSetpoint;
    
    private PIDController xController = new PIDController(1.1, 0, 0); // kp = 1.45, ki = .001
    private final PIDController yController = new PIDController(1.1, 0, 0); // kp = 1
    private final PIDController rotController = new PIDController(1, 0, 0); // kp = 2.5
    private Timer settleTimer;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    public PV_Align(
        CommandSwerveDrivetrain drivetrain,
        VisionGeneral vision,
        int targetId,
        double xSetpoint,
        double ySetpoint,
        double rotSetpoint
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetId = targetId;
        this.xSetpoint = xSetpoint;
        this.ySetpoint = ySetpoint;
        this.rotSetpoint = rotSetpoint;

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
        
        settleTimer = new Timer();
        settleTimer.start();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Vision X", vision.getX(targetId));
        SmartDashboard.putNumber("Vision Y", vision.getY(targetId));
        SmartDashboard.putNumber("Vision Yaw", vision.getYawRad(targetId));

        SmartDashboard.putBoolean("rotSetpoint", rotController.atSetpoint());
        SmartDashboard.putNumber("rot Difference", vision.getYawRad(targetId) - rotController.getSetpoint());
        
        SmartDashboard.putNumber("x Difference", vision.getX(targetId) - xController.getSetpoint());
        SmartDashboard.putBoolean("x Setpoint", xController.atSetpoint());

        SmartDashboard.putNumber("y Difference", vision.getY(targetId) - yController.getSetpoint());
        SmartDashboard.putBoolean("y Setpoint", yController.atSetpoint());

        SmartDashboard.putNumber("settle timer", settleTimer.get());

        if (!vision.hasTarget(targetId)) {
            drivetrain.setControl(new SwerveRequest.Idle());
            return;
        }
        
        double xVel = 0;
        double yVel = 0;
        double rotVel = 0;

        if (!xController.atSetpoint()) {
            xVel = xController.calculate(vision.getX(targetId), xSetpoint);
        } else {
            xVel = 0;
        }
        if (!yController.atSetpoint()) {
            yVel = yController.calculate(vision.getY(targetId), ySetpoint);
        } else {
            yVel = 0;
        }
        if (!rotController.atSetpoint()) {
            rotVel = rotController.calculate(vision.getYawRad(targetId), rotSetpoint);
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
