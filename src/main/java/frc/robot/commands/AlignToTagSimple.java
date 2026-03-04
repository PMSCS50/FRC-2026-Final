package frc.robot.commands;


import static frc.robot.Constants.VisionConstants;
import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;


public class AlignToTagSimple extends Command {
 
    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private static double speedLimiter = 0.2;
    private final CommandSwerveDrivetrain drivetrainSubsystem;
    // private final Supplier<Pose2d> poseProvider;
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(MaxSpeed, 2);
    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("");


    public AlignToTagSimple(CommandSwerveDrivetrain drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        // this.poseProvider = poseProvider;
        xController.setTolerance(0.2);
        addRequirements(drivetrainSubsystem);
    }


    @Override
    public void initialize() {
        xController.reset(0);
    }


    @Override
    public void execute() {
        if (LimelightHelpers.getTV("")) {
            xController.setGoal(0);
            double xOffset = table.getEntry("tx").getDouble(0);
            double xSpeed = xController.calculate(xOffset);
            System.out.println("xSpeed: "+ xSpeed);
            //drive
        } else {
            //brake
        }
    }


    @Override
    public void end(boolean interrupted) {
        // drivetrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        if(xController.atGoal()){
            return true;
        } else {
            return false;
        }
    }

}

