// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.filter.Debouncer;
//import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
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


public class RobotContainer {

    // **************************************************************************************************************
    // DRIVETRAIN CONSTANTS
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double speedLimiter = 0.5;
    private double directionFlipper = -1.0;

    public static double intakeSpeed = .05;
    public static double pivotSpeed = .05;
    public static double shooterSpeed = .05;
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
    private PhotonCamera cam1 = new PhotonCamera("camera1_2585"); //
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
 
    private final VisionSubsystem vision = new VisionSubsystem("meow", drivetrain);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController subjoystick = new CommandXboxController(1);


    private final Shooter shooter = new Shooter(vision);
    private final Intake intake = new Intake();
    private final Climb climb = new Climb();

    // private final L3Climb oliverClimb = new L3Climb();

    /* Path follower */
    private SendableChooser<Command> autoChooser;
    // **************************************************************************************************************








    public RobotContainer() {
        
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        

        
        // NamedCommands.registerCommand("alignToTag", new PV_Align(drivetrain, vision, vision.getBestTarget()));
        // NamedCommands.registerCommand("climbAscend", new ClimbPull(climb));
        // NamedCommands.registerCommand("climbDescend", new ClimbPush(climb));
        // NamedCommands.registerCommand("intakeForward", new IntakeForward(intake));
        // NamedCommands.registerCommand("intakeBack", new IntakeBack(intake));
        // NamedCommands.registerCommand("rotate180Deg", new Rotate180Deg(drivetrain));
        // NamedCommands.registerCommsnd("shoot", new ShootWithoutAim(shooter, vision));
        // NamedCommands.registerCommand("aimShoot", new AimAndShoot(drivetrain, vision, shooter, () -> 0, () -> 0));

        // //for olivers climb if we implement it
        // NamedCommands.registerCommand("L1Ascend", new L1Ascend(oliverClimb));
        // NamedCommands.registerCommand("L1Descend", new L1Descend(oliverClimb));
        
        configureBindings();
    }

    
    private void configureBindings() {

        if (vision.hasTargets()) {
            SmartDashboard.putNumber("Vision X", vision.getX());
            SmartDashboard.putNumber("Vision Y", vision.getY());
            SmartDashboard.putNumber("Vision Yaw", vision.getYawRad());
        }

        // drivetrain.setDefaultCommand(
        //     drivetrain.applyRequest(() -> {
        //         double forward = joystick.getLeftY() * MaxSpeed * speedLimiter * directionFlipper;
        //         double translation = joystick.getLeftX() * MaxSpeed * speedLimiter * directionFlipper;
        //         double turn = joystick.getRightX() * MaxAngularRate * speedLimiter * directionFlipper;

        //         return drive
        //             .withVelocityX(forward)
        //             .withVelocityY(translation)
        //             .withRotationalRate(turn);
        //     })
        // );

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double forward = joystick.getLeftY() * MaxSpeed * speedLimiter * directionFlipper;
                double translation = joystick.getLeftX() * MaxSpeed * speedLimiter * directionFlipper;
                double turn = joystick.getRightX() * MaxAngularRate * speedLimiter * directionFlipper;

                if (joystick.a().getAsBoolean() && vision.hasTarget2(26)) {
                    double kp = 0.03;

                    turn = - kp * MaxAngularRate * vision.getYawRad(); 
                }
                return drive
                    .withVelocityX(forward)
                    .withVelocityY(translation)
                    .withRotationalRate(turn);
            })

        );

        // SUBJOYSTICK 
        // *******************************************************************************************************
        // SHOOTING OPTIONS
        // subjoystick.rightTrigger().whileTrue(
        //         new RunCommand(() -> shooter.setShooterSpeed(subjoystick.getLeftY()), shooter)); // 1. Scala
        // //subjoystick.povUp().whileTrue(new RunCommand(() -> shooter.setShooterSpeed(0.5), shooter));
        // subjoystick.povUp().onFalse(new RunCommand(() -> shooter.stop(), shooter));



        subjoystick.leftTrigger().whileTrue(
            new RunCommand(() -> intake.spinIntake(intakeSpeed), intake));
        subjoystick.leftTrigger().onFalse(
            new RunCommand(() -> intake.stopIntake(), intake));
        subjoystick.leftBumper().onTrue(
            new InstantCommand(() -> intakeSpeed += Math.min(.05, 1)));
        
        subjoystick.rightBumper().onTrue(new InstantCommand(() -> intakeSpeed = .05));
        

        subjoystick.rightTrigger().whileTrue(new DistanceBasedShooting(shooter, vision, 26));

        // subjoystick.povDown().whileTrue(new RunCommand(() -> shooter.setShooterSpeed(0.35), shooter));
        // subjoystick.povDown().onFalse(new RunCommand(() -> shooter.stop(), shooter));
        // subjoystick.povRight().whileTrue(new RunCommand(() -> shooter.setShooterSpeed(0.40), shooter));
        // subjoystick.povRight().onFalse(new RunCommand(() -> shooter.stop(), shooter));
        // subjoystick.povLeft().whileTrue(new RunCommand(() -> shooter.setShooterSpeed(0.45), shooter));
        // subjoystick.povLeft().onFalse(new RunCommand(() -> shooter.stop(), shooter));
        // subjoystick.povUp().whileTrue(new RunCommand(() -> shooter.setShooterSpeed(.5), shooter));
        // subjoystick.povUp().onFalse(new RunCommand(() -> shooter.stop(), shooter));

        // subjoystick.povDown().onFalse(new RunCommand(() -> shooter.stopKicker(), shooter));
        // subjoystick.povUp().onFalse(new RunCommand(() -> shooter.stopKicker(), shooter));
        // subjoystick.povRight().onFalse(new RunCommand(() -> shooter.stopKicker(), shooter));
        // subjoystick.povLeft().onFalse(new RunCommand(() -> shooter.stopKicker(), shooter));

        subjoystick.povUp().onTrue(new InstantCommand (() -> shooterSpeed += Math.min(.05, 1)));
        subjoystick.povRight().onTrue(new InstantCommand (() -> shooterSpeed -= Math.min(.05, 1)));
        subjoystick.povLeft().onTrue(new InstantCommand (() -> shooterSpeed = 0.05));
        subjoystick.povDown().onTrue(new RunCommand(() -> shooter.setShooterSpeed(shooterSpeed)));
        subjoystick.povDown().onFalse(new RunCommand(() -> shooter.stop()));





        subjoystick.a().whileTrue(new RunCommand(() -> intake.movePivot(pivotSpeed), intake));
        subjoystick.a().onFalse(new RunCommand(() -> intake.stopPivot(), intake));
        subjoystick.b().whileTrue(new RunCommand(() -> intake.movePivot(-pivotSpeed), intake));
        subjoystick.b().onFalse(new RunCommand(() -> intake.stopPivot(), intake));



        subjoystick.y().onTrue(new InstantCommand(() -> pivotSpeed += Math.min(.05, 1)));
        subjoystick.x().onTrue(new InstantCommand(() -> pivotSpeed = 0.05));











        // JOYSTICK
        // *******************************************************************************************************


        joystick.b().whileTrue(new RunCommand(() -> this.flipDirection(1.0)));
        joystick.y().whileTrue(new RunCommand(() -> this.flipDirection(-1.0)));

        if(joystick.getLeftX() > 0.5){
            joystick.leftStick().onTrue(new RunCommand(() -> this.setSpeed(1.0)));
        } else {
            joystick.leftStick().onTrue(new RunCommand(() -> this.setSpeed(0.5)));
        }

        
        // PV Align
        joystick.x().onTrue( 
            new PV_Align(drivetrain, vision, 26)  // 17 = target AprilTag ID  
        );
        // ************************************************************************************
        // drivetrain controller | pov buttons | subsystem RunCommand()
        joystick.povUp().whileTrue(new RunCommand(() -> this.setSpeed(1.0)));
        joystick.povRight().whileTrue(new RunCommand(() -> this.setSpeed(0.400)));
        joystick.povLeft().whileTrue(new RunCommand(() -> this.setSpeed(0.200)));
        joystick.povDown().whileTrue(new RunCommand(() -> this.setSpeed(0.1)));
        // ************************************************************************************
        // rollers for subsystems
        // ************************************************************************************
        // subsystems controller | left and right bumpers | subsystem RunCommand()
        
        // ************************************************************************************

    }





    // changing drivetrain speed
    //   crawl, low, mid, high
    public void setSpeed(double spe){
        speedLimiter = spe;
        if(spe == 0.066) SmartDashboard.putString( "Swerve Speed", "CRAWL");
        if(spe == 0.2) SmartDashboard.putString("Swerve Speed", "LOW");
        if(spe == 0.5) SmartDashboard.putString("Swerve Speed", "MID");
        if(spe == 1.0) SmartDashboard.putString("Swerve Speed", "HIGH");
    }

    public void flipDirection(double newDir){
        this.directionFlipper = newDir;
    }
    //  setting the 
    

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        // return null;
        return autoChooser.getSelected();
    }
}
