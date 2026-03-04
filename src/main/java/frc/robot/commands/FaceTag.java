package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class FaceTag extends Command{

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final int targetId;

    private PIDController rotController = new PIDController(0.5,0,0);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    public FaceTag(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, int targetId) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetId = targetId;
            
        
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        rotController.setTolerance(Math.toRadians(5));
        addRequirements(drivetrain);
        addRequirements(vision);

    }
    

    @Override
    public void initialize() {
        rotController.reset();
    }


    @Override
    public void execute() {
        if (!vision.hasTarget(targetId)) {
            drivetrain.setControl(new SwerveRequest.Idle());
            return;
        }

        double rotVel = rotController.calculate(vision.getYawRad(), 0);

        drivetrain.setControl(
            drive.withRotationalRate(-rotVel)

        );
    }

}
