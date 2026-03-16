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
import edu.wpi.first.math.geometry.Pose2d;
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

    //Go to Pathfinder.java for more information.
    //THIS IS NOT FOR REBUILT. It is something potentially for next year and years to come.
    //Go to line 
    private final Pathfinder pathfinder = new Pathfinder(3,3,2 * Math.PI, 2 * Math.PI);

    private final Shooter shooter = new Shooter(vision);
    private final Intake intake = new Intake();
    private final Climb climb = new Climb();

    // private final L3Climb oliverClimb = new L3Climb();

    /* Path follower */
    private SendableChooser<Command> autoChooser;
    // **************************************************************************************************************

    public RobotContainer() {
        
        


        NamedCommands.registerCommand("T-26 Distance Based Shooting", new DistanceBasedShootingTimed(shooter, vision, 26, 4));
        NamedCommands.registerCommand("T-2 Distance Based Shooting", new DistanceBasedShootingTimed(shooter, vision, 2, 4)); // right side
        NamedCommands.registerCommand("T-5 Distance Based Shooting", new DistanceBasedShootingTimed(shooter, vision, 5, 4)); // left side
        

        NamedCommands.registerCommand("Intaking", new Intaking(intake, vision, 3));
        NamedCommands.registerCommand("T-26 PV-Align", new PV_Align(drivetrain, vision, 26, 1.5, 0, 0));
        NamedCommands.registerCommand("Fixed Shooting", new FixedPIDShooting(shooter, 0));


        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    
    private void configureBindings() {

        if (vision.hasTargets()) {
            SmartDashboard.putNumber("Vision X", vision.getX(26));
            SmartDashboard.putNumber("Vision Y", vision.getY(26));
            SmartDashboard.putNumber("Vision Yaw", vision.getYawRad(26));
        }

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double forward = joystick.getLeftY() * MaxSpeed * speedLimiter * directionFlipper;
                double translation = joystick.getLeftX() * MaxSpeed * speedLimiter * directionFlipper;
                double turn = joystick.getRightX() * MaxAngularRate * speedLimiter * directionFlipper;

                return drive
                    .withVelocityX(forward)
                    .withVelocityY(translation)
                    .withRotationalRate(turn);
            })
        );

        // drivetrain.setDefaultCommand(
        //     drivetrain.applyRequest(() -> {
        //         double forward = joystick.getLeftY() * MaxSpeed * speedLimiter * directionFlipper;
        //         double translation = joystick.getLeftX() * MaxSpeed * speedLimiter * directionFlipper;
        //         double turn = joystick.getRightX() * MaxAngularRate * speedLimiter * directionFlipper;
        //         // turn *= 1000;
        //         // turn %= 1;
        //         // turn /= 1000;
        //         // forward *= 1000;
        //         // forward %= 1;
        //         // forward /= 1000;
        //         // translation *= 1000;
        //         // translation %= 1;
        //         // translation /= 1000;
                
        //         if (joystick.a().getAsBoolean() && vision.hasTarget(26)) {
        //             double kp = 0.03;

        //             turn = - kp * MaxAngularRate * vision.getYawRad(26); 
        //         //     turn *= 1000;
        //         //     turn %= 1;
        //         //     turn /= 1000;
        //         }
        //         return drive
        //             .withVelocityX(forward)
        //             .withVelocityY(translation)
        //             .withRotationalRate(turn);
        //     })

        // );

        // SUBJOYSTICK 
        // ******************************************************************************************************
        
    // Triggers and Bumpers
        subjoystick.leftTrigger().whileTrue(new RunCommand(() -> intake.spinIntakePID(intakeSpeed), intake));
        subjoystick.leftTrigger().onFalse(new RunCommand(() -> intake.stopIntake(), intake));
        subjoystick.leftBumper().onTrue(new InstantCommand(() -> intakeSpeed += Math.min(.05, 1)));
        subjoystick.rightBumper().onTrue(new InstantCommand(() -> intakeSpeed = 0.05));
        // subjoystick.rightTrigger().onTrue(new InstantCommand(() -> intakeSpeed = 1));
        subjoystick.rightTrigger().onTrue(new RunCommand(() -> intake.spinIntakePID(-1), intake));


        subjoystick.povDown().whileTrue(new DistanceBasedShooting(shooter, vision, 26));
        

        

        
















        // subjoystick.a().whileTrue(new RunCommand(() -> intake.spinPivotDuty(pivotSpeed), intake));
        // subjoystick.a().onFalse(new RunCommand(() -> intake.stopPivot(), intake));
        // subjoystick.b().whileTrue(new RunCommand(() -> intake.spinPivotDuty(-pivotSpeed), intake));
        // subjoystick.b().onFalse(new RunCommand(() -> intake.stopPivot(), intake));




        subjoystick.x().whileTrue(new RunCommand(() -> intake.spinPivotDuty(.1), intake));
        subjoystick.x().onFalse(new RunCommand(() -> intake.stopPivot(), intake));

        subjoystick.y().whileTrue(new RunCommand(() -> intake.spinPivotDuty(-.1)));
        subjoystick.y().onFalse(new RunCommand(() -> intake.stopPivot(), intake));














        // DRIVETRAIN JOYSTICK
        // *******************************************************************************************************


        joystick.b().whileTrue(new RunCommand(() -> this.flipDirection(1.0)));
        joystick.y().whileTrue(new RunCommand(() -> this.flipDirection(-1.0)));

        if(joystick.getLeftX() > 0.5){
            joystick.leftStick().onTrue(new RunCommand(() -> this.setSpeed(1.0)));
        } else {
            joystick.leftStick().onTrue(new RunCommand(() -> this.setSpeed(0.5)));
        }

        
        // PV Align
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

        
        /*
        Here, we use Pathfinder to create a path to a specific Pose2d, even on teleop.
        Then it will follow the path as long as the button is held
        Releasing button will stop the path following and allow for manual control again.
        We can basically create waypoints on the field and map them to certain buttons.
        This is crazy because if there is a setpoint in future games that we want to be able to quickly and easily drive to,
            we can just make a button for it and use Pathfinder to get there.

        In the context of REBUILT, we could use this in teleop 
        */
        joystick.rightTrigger().whileTrue(pathfinder.makePathTo(new Pose2d(3, 3, new Rotation2d(0))));
        
        
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
