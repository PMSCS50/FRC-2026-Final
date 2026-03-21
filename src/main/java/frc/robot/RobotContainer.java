// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants.VisionConstants;
//import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.L3Climb;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.Constants;


public class RobotContainer {

    // **************************************************************************************************************
    // DRIVETRAIN CONSTANTS
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double speedLimiter = 0.5;
    private double directionFlipper = VisionConstants.getDirectionFlipper();

    public static double intakeSpeed = 1;
    public static double pivotSpeed = .05;
    public static double shooterSpeed = .01;
    // **************************************************************************************************************

    // **************************************************************************************************************
    // EXTRA SETUP - I GOT NO CLUE
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 2% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // **************************************************************************************************************

    // **************************************************************************************************************
    // ACTUAL IMPORTANT STUFF
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
 
    private final VisionSubsystem vision = new VisionSubsystem("meow", drivetrain);

    private final CommandXboxController joystick = new CommandXboxController(0);
    public static final CommandXboxController subjoystick = new CommandXboxController(1);

    private final Shooter shooter = new Shooter(vision);
    private final Intake intake = new Intake();
    private final Climb climb = new Climb();

    /* Path follower */
    private SendableChooser<Command> autoChooser;

    double turningSpeed = 0; // for speed scaling
    // **************************************************************************************************************


    public RobotContainer() {
// Distance Based Shooting
    // Middle Tag
        NamedCommands.registerCommand("4 sec Middle Distance Based Shooting", 
            new DistanceBasedShooting(shooter, vision, VisionConstants.getMiddleTagId()).withTimeout(4)); // 4 seconds
        NamedCommands.registerCommand("6 sec Middle Distance Based Shooting", 
            new DistanceBasedShooting(shooter, vision, VisionConstants.getMiddleTagId()).withTimeout(6)); // 6 seconds
        NamedCommands.registerCommand("8 sec Middle Distance Based Shooting", 
            new DistanceBasedShooting(shooter, vision, VisionConstants.getMiddleTagId()).withTimeout(8)); // 8 seconds
    // Left Tag    
        NamedCommands.registerCommand("4 sec Left Distance Based Shooting", 
            new DistanceBasedShooting(shooter, vision, VisionConstants.getLeftTagId()).withTimeout(4)); // 4 seconds
        NamedCommands.registerCommand("6 sec Left Distance Based Shooting", 
            new DistanceBasedShooting(shooter, vision, VisionConstants.getLeftTagId()).withTimeout(6)); // 6 seconds
        NamedCommands.registerCommand("8 sec Left Distance Based Shooting", 
            new DistanceBasedShooting(shooter, vision, VisionConstants.getLeftTagId()).withTimeout(8)); // 8 seconds
    // Right Tag
        NamedCommands.registerCommand("4 sec Right Distance Based Shooting", 
            new DistanceBasedShooting(shooter, vision, VisionConstants.getRightTagId()).withTimeout(4)); // 4 seconds
        NamedCommands.registerCommand("6 sec Right Distance Based Shooting", 
            new DistanceBasedShooting(shooter, vision, VisionConstants.getRightTagId()).withTimeout(6)); // 6 seconds
        NamedCommands.registerCommand("8 sec Right Distance Based Shooting", 
            new DistanceBasedShooting(shooter, vision, VisionConstants.getRightTagId()).withTimeout(8)); // 8 seconds

// Intaking
        NamedCommands.registerCommand("3.5 sec Intaking", new Intaking(intake).withTimeout(3.5));
        NamedCommands.registerCommand("4 sec Intaking", new Intaking(intake).withTimeout(4));
        NamedCommands.registerCommand("6 sec Intaking", new Intaking(intake).withTimeout(6));

// Orientation
    // Middle
        NamedCommands.registerCommand("0.5 sec Middle Orientation", new PV_Orient(drivetrain, vision, VisionConstants.getMiddleTagId(), 0).withTimeout(1));
        NamedCommands.registerCommand("1 sec Middle Orientation", new PV_Orient(drivetrain, vision, VisionConstants.getMiddleTagId(), 0).withTimeout(1));
    // Left
        NamedCommands.registerCommand("0.5 sec Left Orientation", new PV_Orient(drivetrain, vision, VisionConstants.getLeftTagId(), 0).withTimeout(1));
        NamedCommands.registerCommand("1 sec Left Orientation", new PV_Orient(drivetrain, vision, VisionConstants.getLeftTagId(), 0).withTimeout(1));
    // RIght
        NamedCommands.registerCommand("0.5 sec Right Orientation", new PV_Orient(drivetrain, vision, VisionConstants.getRightTagId(), 0).withTimeout(1));
        NamedCommands.registerCommand("1 sec Right Orientation", new PV_Orient(drivetrain, vision, VisionConstants.getRightTagId(), 0).withTimeout(1));

// Alignment
    // Middle

    // Left

    // Right
        NamedCommands.registerCommand("Left Shoot PV-Align", new PV_Align(drivetrain, vision, VisionConstants.getLeftTagId(), 0, 0, 0));
        NamedCommands.registerCommand("T-26 PV-Align", new PV_Align(drivetrain, vision, VisionConstants.getMiddleTagId(), 1.5, 0, 0));
        NamedCommands.registerCommand("Fixed Shooting", new FixedPIDShooting(shooter, 0));

// Pivoting

        NamedCommands.registerCommand("Forward Pivoting", new Pivoting(intake, true).withTimeout(1));
        NamedCommands.registerCommand("Pivoting Back", new Pivoting(intake, false).withTimeout(1));

// Configuring
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        CameraServer.startAutomaticCapture();
        configureBindings();
    }

    
    private void configureBindings() {
        if (vision.hasTargets()) {
            SmartDashboard.putNumber("Vision X", vision.getX(VisionConstants.getMiddleTagId()));
            SmartDashboard.putNumber("Vision Y", vision.getY(VisionConstants.getMiddleTagId()));
            SmartDashboard.putNumber("Vision Yaw", vision.getYawRad(VisionConstants.getMiddleTagId()));
        }
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double forward =  joystick.getLeftY() * MaxSpeed * directionFlipper * speedLimiter;
                double translation = joystick.getLeftX() * MaxSpeed * directionFlipper * speedLimiter;
                double turn = joystick.getRightX() * MaxAngularRate * speedLimiter* -1;
                return drive
                    .withVelocityX(forward)
                    .withVelocityY(translation)
                    .withRotationalRate(turn);
            })
        );

        joystick.a().whileTrue(
            drivetrain.applyRequest(() -> {
                double forward = joystick.getLeftY() * MaxSpeed * speedLimiter * directionFlipper;
                double translation = joystick.getLeftX() * MaxSpeed * speedLimiter * directionFlipper;
                double turn = vision.hasTarget(VisionConstants.getMiddleTagId()) ? -vision.getTargetYaw(VisionConstants.getMiddleTagId()) * .03 * MaxAngularRate :
                    vision.hasTarget(VisionConstants.getLeftTagId()) && !vision.hasTarget(VisionConstants.getMiddleTagId()) ? -vision.getTargetYaw(VisionConstants.getLeftTagId()) * .03 * MaxAngularRate : 
                    vision.hasTarget(VisionConstants.getRightTagId()) && !vision.hasTarget(VisionConstants.getMiddleTagId()) ? -vision.getTargetYaw(VisionConstants.getRightTagId()) * .03 * MaxAngularRate : 0;

                return drive
                    .withVelocityX(forward)
                    .withVelocityY(translation)
                    .withRotationalRate(turn);
            })
        );



// SUBJOYSTICK 
// ******************************************************************************************************
    // Triggers and Bumpers
        subjoystick.leftTrigger().whileTrue(new RunCommand(() -> intake.spinIntakePID(1), intake));
        subjoystick.leftTrigger().onFalse(new RunCommand(() -> intake.stopIntake(), intake));

        subjoystick.leftBumper().whileTrue(new RunCommand(() -> intake.spinIntakePID(-1), intake));
        subjoystick.leftBumper().onFalse(new RunCommand(() -> intake.stopIntake(), intake));

        subjoystick.rightTrigger().onTrue(new Pivoting(intake, true));
        subjoystick.rightBumper().onTrue(new Pivoting(intake, false));

    // POV Controls    
        subjoystick.povDown().or(subjoystick.povDownRight()).or(subjoystick.povDownLeft()).whileTrue(new DistanceBasedShooting(shooter, vision, 
            VisionConstants.getMiddleTagId(), VisionConstants.getLeftTagId(), VisionConstants.getRightTagId())); 

        
        subjoystick.povLeft().whileTrue(new FixedPIDShooting(shooter, 3)); // side of climb
        subjoystick.povUp().whileTrue(new FixedPIDShooting(shooter, 2.5)); // side of slope

    // Letters
        // subjoystick.a()
        subjoystick.b().onTrue(new RunCommand(() -> intake.resetPivot(), intake));
        subjoystick.x().whileTrue(new RunCommand(() -> intake.spinPivotDuty(.3), intake));
        subjoystick.x().onFalse(new RunCommand(() -> intake.stopPivot(), intake));
        subjoystick.y().whileTrue(new RunCommand(() -> intake.spinPivotDuty(-.3), intake));
        subjoystick.y().onFalse(new RunCommand(() -> intake.stopPivot(), intake));
        


// DRIVETRAIN JOYSTICK
// *******************************************************************************************************

    // Triggers and Bumpers
        joystick.leftTrigger().whileTrue(new RunCommand(() -> intake.spinIntakePID(1), intake));
        joystick.leftTrigger().onFalse(new RunCommand(() -> intake.stopIntake(), intake));
        joystick.rightTrigger().whileTrue(new RunCommand(() -> intake.spinIntakePID(-1), intake));
        joystick.rightTrigger().onFalse(new RunCommand(() -> intake.stopIntake(), intake));

        joystick.leftBumper().onTrue(new InstantCommand(() -> this.setSpeed(speedLimiter-.1)));
        joystick.rightBumper().onTrue(new InstantCommand(() -> this.setSpeed(speedLimiter+.1)));

    // POV Controls
        // joystick.povUp()
        // joystick.povRight()
        // joystick.povLeft()
        // joystick.povUp()


        // joystick.leftBumper().onTrue(new InstantCommand() -> driveSpeed)


        // joystick.povUp().whileTrue(new RunCommand(() -> this.setSpeed(1.0)));
        // joystick.povRight().whileTrue(new RunCommand(() -> this.setSpeed(0.500)));
        // joystick.povLeft().whileTrue(new RunCommand(() -> this.setSpeed(0.200)));
        // joystick.povDown().whileTrue(new RunCommand(() -> this.setSpeed(0.1)));

    // Letters
        // joystick.a()
        joystick.b().whileTrue(new RunCommand(() -> this.flipDirection(1.0)));
        joystick.x().whileTrue(new PV_Align(drivetrain, vision, VisionConstants.getMiddleTagId(), 1.5, 0, 0));
        joystick.y().whileTrue(new RunCommand(() -> this.flipDirection(-1.0)));

 


 
        
        // joystick.a().whileTrue(new AimToPose(drivetrain, vision, VisionConstants.getHubPose(), xInput, yInput));
        

        


        
        /*
        Here, we use Pathfinder to create a path to a specific Pose2d, even on teleop.
        Then it will follow the path as long as the button is held
        Releasing button will stop the path following and allow for manual control again.
        We can basically create waypoints on the field and map them to certain buttons.
        This is crazy because if there is a setpoint in future games that we want to be able to quickly and easily drive to,
            we can just make a button for it and use Pathfinder to get there.

        In the context of REBUILT, we could use this in teleop to align to climb perfectly
        */
        // joystick.rightTrigger().whileTrue(pathfinder.makePathTo(new Pose2d(3, 3, new Rotation2d(0))));

        
        // Command climbPath = pathfinder.makePathTo(Constants.ClimbConstants.getClimbPose(), drivetrain, vision);

        // joystick.rightTrigger().onTrue(climbPath);

        // joystick.rightTrigger().onFalse(Commands.runOnce(climbPath::cancel));
        
        
    }




    // changing drivetrain speed
    //   crawl, low, mid, high
    public void setSpeed(double speed) {
        if (speed < 0.1) {
            speed = 0.1;
        }
        if(speed <= .25) {
            SmartDashboard.putString( "Swerve Speed", "CRAWL");
            turningSpeed = .25;
        } else {
            turningSpeed = joystick.getRightX() * MaxAngularRate * speedLimiter * directionFlipper;
            if(speed == 0.2) SmartDashboard.putString("Swerve Speed", "LOW");
            if(speed == 0.5) SmartDashboard.putString("Swerve Speed", "MID");
            if(speed == 1.0) SmartDashboard.putString("Swerve Speed", "HIGH");
        }
        speedLimiter = speed;
        
    }



    public void flipDirection(double newDir){
        this.directionFlipper = newDir;
    }
    

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        // return null;
        return autoChooser.getSelected();
    }

    public Intake getIntake() {
        return intake;
    }
    public Climb getClimb() {
        return climb;
    }
    public Shooter getShooter() {
        return shooter;
    }


}
