// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.hardware.Pigeon2;
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
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.subsystems.LLSubsystem;
import frc.robot.subsystems.Pivot;
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

    public static double intakeSpeed = 0.5;
    public static double pivotSpeed = .05;
    public static double shooterSpeed = .01;
    // **************************************************************************************************************

    // **************************************************************************************************************
    // EXTRA SETUP - I GOT NO CLUE
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 2% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake xBrake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);
    

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // **************************************************************************************************************

    // **************************************************************************************************************
    // ACTUAL IMPORTANT STUFF
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
 
    private final VisionSubsystem vision = new VisionSubsystem("meow", drivetrain);
    private final LLSubsystem LLVision = new LLSubsystem(drivetrain, VisionConstants.limelightName);
    private final CommandXboxController joystick = new CommandXboxController(0);
    public static final CommandXboxController subjoystick = new CommandXboxController(1);

    private final Shooter shooter = new Shooter(LLVision);
    private final Intake intake = new Intake();
    private final Climb climb = new Climb();
    private final Pivot pivot = new Pivot();

    /* Path follower */
    private SendableChooser<Command> autoChooser;

    double turningSpeed = 0; // for speed scaling
    // **************************************************************************************************************


    public RobotContainer() {
       NamedCommands.registerCommand("Distance Based Shooting", new DistanceBasedShooting(shooter, LLVision).withTimeout(4));
// Intaking
        NamedCommands.registerCommand("3.5 sec Intaking", new Intaking(intake).withTimeout(3.5));
        NamedCommands.registerCommand("4 sec Intaking", new Intaking(intake).withTimeout(4));
        NamedCommands.registerCommand("6 sec Intaking", new Intaking(intake).withTimeout(6));

// Alignment
    // Middle

    // Left

    // Right
        NamedCommands.registerCommand("Left Shoot PV-Align", new PV_Align(drivetrain, vision, VisionConstants.getLeftTagId(), 0, 0, 0));
        NamedCommands.registerCommand("T-26 PV-Align", new PV_Align(drivetrain, vision, VisionConstants.getMiddleTagId(), 1.5, 0, 0));
        // NamedCommands.registerCommand("Fixed Shooting", new FixedPIDShooting(shooter, 0));

// Pivoting

        NamedCommands.registerCommand("Forward Pivoting 30%", new Pivoting(pivot, true).withTimeout(.5));
        NamedCommands.registerCommand("Pivoting Back 30%" , new Pivoting(pivot, false ).withTimeout(.5));
        NamedCommands.registerCommand("Forward Pivoting 10%", new Pivoting(pivot, true).withTimeout(1.5));
        NamedCommands.registerCommand("Pivoting Back 10%" , new Pivoting(pivot, false).withTimeout(1.5));

        NamedCommands.registerCommand("Fixed Based Shooting Auton", new FixedPIDShooting(shooter, 3.3).withTimeout(4));

        


// Configuring
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        CameraServer.startAutomaticCapture();
        configureBindings();
    }

    
    private void configureBindings() {

        SmartDashboard.putNumber("Shooting Speed", shooterSpeed);
        SmartDashboard.putBoolean("Has Targets", vision.hasTargets());

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


// SUBJOYSTICK 
// ******************************************************************************************************
    // Triggers and Bumpers
        subjoystick.leftTrigger().whileTrue(new RunCommand(() -> intake.spinIntakePID(1), intake));
        subjoystick.leftTrigger().onFalse(new RunCommand(() -> intake.stopIntake(), intake)); 
        subjoystick.leftBumper().whileTrue(new RunCommand(() -> intake.spinIntakePID(-1), intake));
        subjoystick.leftBumper().onFalse(new RunCommand(() -> intake.stopIntake(), intake));

        subjoystick.rightTrigger().whileTrue(new Pivoting(pivot, true));
        subjoystick.rightBumper().whileTrue(new Pivoting(pivot, false));

    // POV Controls    
        subjoystick.povUp().or(subjoystick.povUpLeft()).or(subjoystick.povUpRight()).whileTrue(new FixedPIDShooting(shooter,1.4));
        subjoystick.povDown().or(subjoystick.povDownLeft()).or(subjoystick.povDownRight()).whileTrue(new DistanceBasedShooting(shooter, LLVision));
       //  subjoystick.povDown().or(subjoystick.povDownLeft()).or(subjoystick.povDownRight()).whileTrue(new FixedPIDShooting(shooter, 3.3));
        

        


        // subjoystick.povLeft()
        // subjoystick.povRight()

    // Letters
        subjoystick.a().whileTrue(new FixedPIDShooting(shooter, 5));
        subjoystick.b().onTrue(new RunCommand(() -> pivot.resetPivot(), pivot));
        subjoystick.x().whileTrue(new RunCommand(() -> pivot.spinPivotDuty(.3), pivot));
        subjoystick.x().onFalse(new RunCommand(() -> pivot.stopPivot(), pivot));
        subjoystick.y().whileTrue(new RunCommand(() -> pivot.spinPivotDuty(-.3), pivot));
        subjoystick.y().onFalse(new RunCommand(() -> pivot.stopPivot(), pivot));
        


// DRIVETRAIN JOYSTICK
// *******************************************************************************************************

    // Triggers and Bumpers
        joystick.leftTrigger().whileTrue(
            Commands.parallel(
                new RunCommand(() -> intake.spinIntakePID(1), intake),
                new RunCommand(() -> shooter.spinKickersAgain(-.6), shooter)
            )
        );
        joystick.leftTrigger().onFalse(
            Commands.parallel(
                new RunCommand(() -> intake.stopIntake(), intake),
                new RunCommand(() -> shooter.stopKicker(), shooter)
            )
        );
        joystick.rightTrigger().whileTrue(new RunCommand(() -> intake.spinIntakePID(-1), intake));
        joystick.rightTrigger().onFalse(new RunCommand(() -> intake.stopIntake(), intake));

        joystick.leftBumper().onTrue(new InstantCommand(() -> this.setSpeed(speedLimiter-.1)));
        joystick.rightBumper().onTrue(new InstantCommand(() -> this.setSpeed(speedLimiter+.1)));


        

    // POV Controls
        // joystick.povUp()
        // joystick.povRight()
        // joystick.povLeft()
        // joystick.povUp()

        // joystick.povUp().whileTrue(new RunCommand(() -> this.setSpeed(1.0)));
        // joystick.povRight().whileTrue(new RunCommand(() -> this.setSpeed(0.500)));
        // joystick.povLeft().whileTrue(new RunCommand(() -> this.setSpeed(0.200)));
        // joystick.povDown().whileTrue(new RunCommand(() -> this.setSpeed(0.1)));

    // Letters
        joystick.a().whileTrue(new AlignToHub(drivetrain, LLVision));
        joystick.b().whileTrue(new RunCommand(() -> this.flipDirection(1.0)));
        joystick.x().whileTrue(drivetrain.applyRequest(() -> xBrake));
        joystick.y().whileTrue(new RunCommand(() -> this.flipDirection(-1.0)));

 


 
        
        

        


        
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

    public void setShooterSpeed(double speed) {
        if (speed < 0.01) {
            speed = 0.01;
        } else if (speed > 1) {
            shooterSpeed = 1;
        } else {
            shooterSpeed = speed;
        }
    }


    // changing drivetrain speed
    //   crawl, low, mid, high
    public void setSpeed(double speed) {
        if (speed < 0.1) {
            speed = 0.1;
            SmartDashboard.putString( "Swerve Speed", "CRAWL");
        }
        if(speed <= .25) {
            turningSpeed = .25;
            SmartDashboard.putString("Swerve Speed", "LOW");
        } else {
            turningSpeed = joystick.getRightX() * MaxAngularRate * speedLimiter * directionFlipper;
            if (speed <= 0.3) SmartDashboard.putString("Swerve Speed", "LOW");
            else if (speed <= 0.5) SmartDashboard.putString("Swerve Speed", "MID");
            else SmartDashboard.putString("Swerve Speed", "HIGH");
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
    public Pivot getPivot() {
        return pivot;
    }


}
