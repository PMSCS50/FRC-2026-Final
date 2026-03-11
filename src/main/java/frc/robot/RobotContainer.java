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
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    private final Telemetry logger = new Telemetry(MaxSpeed); // What does this actually do?

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
        
        // CameraServer.startAutomaticCapture();        
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

        joystick.a().whileTrue(new FaceTag(drivetrain, vision, 17));

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

                if (joystick.a().getAsBoolean() && vision.hasTargets() && vision.getTargetId() == 4) {
                    double kp = 0.03;

                    turn = - kp * MaxAngularRate * vision.getYawRad(); 
                }
                return drive
                    .withVelocityX(forward)
                    .withVelocityY(translation)
                    .withRotationalRate(turn);
            })

        );


        // *******************************************************************************************************
        // SHOOTING OPTIONS
        // subjoystick.rightTrigger().whileTrue(
        //         new RunCommand(() -> shooter.setShooterSpeed(subjoystick.getLeftY()), shooter)); // 1. Scala
        // //subjoystick.povUp().whileTrue(new RunCommand(() -> shooter.setShooterSpeed(0.5), shooter));
        //subjoystick.povUp().onFalse(new RunCommand(() -> shooter.stop(), shooter));


        // subjoystick.povDown().whileTrue(new RunCommand(() -> shooter.setShooterSpeed(0.20), shooter));
        // subjoystick.povDown().onFalse(new RunCommand(() -> shooter.stop(), shooter));
        // subjoystick.povRight().whileTrue(new RunCommand(() -> shooter.setShooterSpeed(0.5), shooter));
        // subjoystick.povRight().onFalse(new RunCommand(() -> shooter.stop(), shooter));
        // subjoystick.povLeft().whileTrue(new RunCommand(() -> shooter.setShooterSpeed(0.75), shooter));
        // subjoystick.povLeft().onFalse(new RunCommand(() -> shooter.stop(), shooter));
        // subjoystick.povUp().whileTrue(new RunCommand(() -> shooter.setShooterSpeed(1), shooter));
        // subjoystick.povUp().onFalse(new RunCommand(() -> shooter.stop(), shooter));

        // subjoystick.b().whileTrue(new AimAndShoot(drivetrain, vision, shooter, () -> 0,() -> 0));

        subjoystick.povDown().whileTrue(new RunCommand(() -> intake.spinIntake(0.25), intake));
        subjoystick.povDown().onFalse(new RunCommand(() -> intake.stopIntake(), intake));
        subjoystick.povRight().whileTrue(new RunCommand(() -> intake.spinIntake(0.50), intake));
        subjoystick.povRight().onFalse(new RunCommand(() -> intake.stopIntake(), intake));
        subjoystick.povLeft().whileTrue(new RunCommand(() -> intake.spinIntake(0.75), intake));
        subjoystick.povLeft().onFalse(new RunCommand(() -> intake.stopIntake(), intake));
        subjoystick.povUp().whileTrue(new RunCommand(() -> intake.spinIntake(1), intake));
        subjoystick.povUp().onFalse(new RunCommand(() -> intake.stopIntake(), intake));

       
        



        
              
        subjoystick.leftTrigger().onFalse(new RunCommand(() -> shooter.stop(), shooter));

        subjoystick.leftBumper().whileTrue(new RunCommand(() -> intake.spinIntake(-1), intake));
        subjoystick.leftBumper().onFalse(new RunCommand(() -> intake.stopIntake(), intake));
        subjoystick.rightBumper().whileTrue(new RunCommand(() -> intake.spinIntake(-0.8), intake));
        subjoystick.rightBumper().onFalse(new RunCommand(() -> intake.stopIntake(), intake));


        // subjoystick.x().whileTrue(new RunCommand(() -> intake.deployIntake(.03), intake));
        // subjoystick.x().whileTrue(new RunCommand(() -> intake.deployIntake(0), intake));
        // subjoystick.y().whileTrue(new RunCommand(() -> intake.deployIntake(-.03), intake));
        // subjoystick.y().whileTrue(new RunCommand(() -> intake.deployIntake(0), intake));
        

        
        
        /* 

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double forward = joystick.getLeftY() * MaxSpeed * speedLimiter * directionFlipper;
                double translation = joystick.getLeftX() * MaxSpeed * speedLimiter * directionFlipper;
                double turn = joystick.getRightX() * MaxAngularRate * speedLimiter * directionFlipper;

                if (joystick.a().getAsBoolean() && vision.hasTargets() && vision.getTargetID() == 18) {
                    double kp = 0.03;

                    turn = - kp * MaxAngularRate * Math.toRadians(vision.getTargetYaw()); 
                }
                return drive
                    .withVelocityX(forward)
                    .withVelocityY(translation)
                    .withRotationalRate(turn);
            })

        );
        */
        //joystick.x().onTrue(new PV_Align(drivetrain, vision));

        
        // drivetrain.setDefaultCommand(
        //     drivetrain.applyRequest(() -> {
        //         double forward = joystick.getLeftY() * MaxSpeed * speedLimiter * directionFlipper;
        //         double translation = joystick.getLeftX() * MaxSpeed * speedLimiter * directionFlipper;
        //         double turn = joystick.getRightX() * MaxAngularRate * speedLimiter * 1.5;
        //         if (joystick.a().getAsBoolean() && vision.hasTargets()) {
        //             double kp = 0.03;
        //             turn = -vision.getTargetYaw() * kp * MaxAngularRate;
        //         }
            
        //         return drive
        //             .withVelocityX(forward)
        //             .withVelocityY(translation)
        //             .withRotationalRate(turn);
                
                
        //     })
        // );

        joystick.b().whileTrue(new RunCommand(() -> this.flipDirection(1.0)));
        joystick.y().whileTrue(new RunCommand(() -> this.flipDirection(-1.0)));

        if(joystick.getLeftX() > 0.5){
            joystick.leftStick().onTrue(new RunCommand(() -> this.setSpeed(1.0)));
        } else {
            joystick.leftStick().onTrue(new RunCommand(() -> this.setSpeed(0.5)));
        }

        // align to reef tag
        //joystick.leftStick().onTrue(new AlignToReefTagRelative(drivetrain, false));
        // subjoystick.a().whileTrue(
        //     new RunCommand(() -> {
        //         if (climb.isPressed()) {
        //             climb.runClimb();
        //         } else {
        //             climb.stopClimb();
        //         }
        //     }, climb)
        // );
        
        // subjoystick.b().whileTrue(new RunCommand(() -> climb.runClimb(), climb));
        // subjoystick.b().onFalse(new RunCommand(() -> climb.stopClimb(), climb));
        
        
        joystick.x().onTrue( 
            new PV_Align(drivetrain, vision, 4)  // 17 = target AprilTag ID  
        );
        
        
        // ************************************************************************************
        // drivetrain controller | pov buttons | subsystem RunCommand()
        joystick.povUp().whileTrue(new RunCommand(() -> this.setSpeed(1.0)));
        joystick.povRight().whileTrue(new RunCommand(() -> this.setSpeed(0.400)));
        joystick.povLeft().whileTrue(new RunCommand(() -> this.setSpeed(0.200)));
        joystick.povDown().whileTrue(new RunCommand(() -> this.setSpeed(0.1)));
        // ************************************************************************************
        //rollers for subsystems
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
