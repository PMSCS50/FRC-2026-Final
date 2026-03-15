package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.VisionConstants;

//Alternate AimAndShoot that uses PhotonUtils.getYawToPose() and PhotonVision.getDistanceToPose().
//With this, we can aim directly at the hub instead of relying on AprilTags
//However, we need to make sure our fieldToRobot updating is correct.

public class AimAndShoot2 extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final VisionSubsystem vision;
    private final PIDController rotController;

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    public AimAndShoot2(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.shooter = shooter;
        rotController = new PIDController(10,0.05, 0.005); //PID tuned in advantagekit
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain, vision, shooter);
    }

    @Override
    public void initialize() {
        rotController.setTolerance(0.000001);
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
        
        if (rotController.atSetpoint()) {
            shooter.rpmControl(distance);
            if (shooter.atCorrectRPM(distance)) {
                shooter.spinKickers();
            }
        } else {
            shooter.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            drive.withVelocityX(0)
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