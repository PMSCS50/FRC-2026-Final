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
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.DistanceBasedShooting;
import frc.robot.commands.FixedPIDShooting;
import frc.robot.commands.PV_Align;
import frc.robot.commands.PV_Orient;
import frc.robot.commands.Pivoting;
import frc.robot.commands.Intaking;
import frc.robot.pathfinding.Pathmaster;



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
    private final LLSubsystem LLVision = new LLSubsystem(drivetrain, "limelight-meowlit");

    private final CommandXboxController joystick = new CommandXboxController(0);
    public static final CommandXboxController subjoystick = new CommandXboxController(1);

    private final Shooter shooter = new Shooter(LLVision);
    private final Intake intake = new Intake();
    private final Climb climb = new Climb();
    private final Pivot pivot = new Pivot();

    Pathmaster pathmaster = new Pathmaster(drivetrain, MaxSpeed, 5, MaxAngularRate, 2*Math.PI);

    /* Path follower */
    private SendableChooser<Command> autoChooser;

    double turningSpeed = 0; // for speed scaling
    // **************************************************************************************************************


    public RobotContainer() {
// Distance Based Shooting
    // Middle Tag
    //     NamedCommands.registerCommand("4 sec Middle Distance Based Shooting", 
    //         new DistanceBasedShooting(shooter, vision, VisionConstants.getMiddleTagId()).withTimeout(4)); // 4 seconds
    //     NamedCommands.registerCommand("6 sec Middle Distance Based Shooting", 
    //         new DistanceBasedShooting(shooter, vision, VisionConstants.getMiddleTagId()).withTimeout(6)); // 6 seconds
    //     NamedCommands.registerCommand("8 sec Middle Distance Based Shooting", 
    //         new DistanceBasedShooting(shooter, vision, VisionConstants.getMiddleTagId()).withTimeout(8)); // 8 seconds
    // // Left Tag    
    //     NamedCommands.registerCommand("4 sec Left Distance Based Shooting", 
    //         new DistanceBasedShooting(shooter, vision, VisionConstants.getLeftTagId()).withTimeout(4)); // 4 seconds
    //     NamedCommands.registerCommand("6 sec Left Distance Based Shooting", 
    //         new DistanceBasedShooting(shooter, vision, VisionConstants.getLeftTagId()).withTimeout(6)); // 6 seconds
    //     NamedCommands.registerCommand("8 sec Left Distance Based Shooting", 
    //         new DistanceBasedShooting(shooter, vision, VisionConstants.getLeftTagId()).withTimeout(8)); // 8 seconds
    // // Right Tag
    //     NamedCommands.registerCommand("4 sec Right Distance Based Shooting", 
    //         new DistanceBasedShooting(shooter, vision, VisionConstants.getRightTagId()).withTimeout(4)); // 4 seconds
    //     NamedCommands.registerCommand("6 sec Right Distance Based Shooting", 
    //         new DistanceBasedShooting(shooter, vision, VisionConstants.getRightTagId()).withTimeout(6)); // 6 seconds
    //     NamedCommands.registerCommand("8 sec Right Distance Based Shooting", 
    //         new DistanceBasedShooting(shooter, vision, VisionConstants.getRightTagId()).withTimeout(8)); // 8 seconds

       //  NamedCommands.registerCommand("Fixed Shooting Left Shoot", new FixedPIDShooting(shooter, 1.5));

       NamedCommands.registerCommand("Distance Based Shooting", new DistanceBasedShooting(shooter, LLVision).withTimeout(4));



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
        // NamedCommands.registerCommand("Fixed Shooting", new FixedPIDShooting(shooter, 0));

// Pivoting

        NamedCommands.registerCommand("Forward Pivoting 30%", new Pivoting(pivot, true).withTimeout(.5));
        NamedCommands.registerCommand("Pivoting Back 30%" , new Pivoting(pivot, false ).withTimeout(.5));
        NamedCommands.registerCommand("Forward Pivoting 10%", new Pivoting(pivot, true).withTimeout(1.5));
        NamedCommands.registerCommand("Pivoting Back 10%" , new Pivoting(pivot, false).withTimeout(1.5));


// Configuring
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        CameraServer.startAutomaticCapture();
        configureBindings();
    }

    
    private void configureBindings() {

        SmartDashboard.putNumber("Shooting Speed", shooterSpeed);
        SmartDashboard.putBoolean("Has Targets", vision.hasTargets());
        
        if (vision.hasTargets()) {
            SmartDashboard.putNumber("Vision Distance", vision.getDistance(VisionConstants.getMiddleTagId()));
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





// SUBJOYSTICK 
// ******************************************************************************************************
    // Triggers and Bumpers
        subjoystick.leftTrigger().whileTrue(new RunCommand(() -> intake.spinIntakePID(1), intake));
        subjoystick.leftTrigger().onFalse(new RunCommand(() -> intake.stopIntake(), intake));
        subjoystick.leftBumper().whileTrue(new RunCommand(() -> intake.spinIntakePID(-1), intake));
        subjoystick.leftBumper().onFalse(new RunCommand(() -> intake.stopIntake(), intake));

        // subjoystick.leftBumper().onTrue(new InstantCommand(() -> intakeSpeed -= 0.05));
        // subjoystick.rightBumper().onTrue(new InstantCommand(() -> intakeSpeed += 0.05));
        // subjoystick.leftTrigger().whileTrue(new RunCommand(() -> intake.spinIntakePID(intakeSpeed), intake));
        // subjoystick.leftTrigger().onFalse(new RunCommand(() -> intake.stopIntake(), intake));

       

        subjoystick.rightTrigger().whileTrue(new Pivoting(pivot, true));
        subjoystick.rightBumper().whileTrue(new Pivoting(pivot, false));

    // POV Controls    
        // subjoystick.povDown().or(subjoystick.povDownRight()).or(subjoystick.povDownLeft()).whileTrue(new DistanceBasedShooting(shooter, vision, 
        //     VisionConstants.getMiddleTagId(), VisionConstants.getLeftTagId(), VisionConstants.getRightTagId())); 

        
        // subjoystick.povLeft().whileTrue(new FixedPIDShooting(shooter, 3)); // side of climb
        // subjoystick.povUp().whileTrue(new FixedPIDShooting(shooter, 2.5)); // side of slope
        // subjoystick.povLeft().onTrue(new RunCommand(() -> LLVision.setPigeon()));
        subjoystick.povUp().or(subjoystick.povUpLeft()).or(subjoystick.povUpRight()).onTrue(new FixedPIDShooting(shooter,1.23));
        // subjoystick.povUp().or(subjoystick.povUpLeft()).or(subjoystick.povUpRight()).whileTrue(new FixedPIDShooting(shooter, 2.25));
        // subjoystick.povUp().or(subjoystick.povUpLeft()).or(subjoystick.povUpRight()).onTrue(new InstantCommand(() -> shooterSpeed += .01));
        // subjoystick.povDown().or(subjoystick.povDownRight()).or(subjoystick.povDownLeft()).whileTrue(new FixedPIDShooting(shooter,() -> shooterSpeed));
        // subjoystick.povDown().or(subjoystick.povDownRight()).or(subjoystick.povDownLeft()).whileTrue(new FixedPIDShooting(shooter, () -> shooterSpeed));
        subjoystick.povDown().or(subjoystick.povDownRight()).or(subjoystick.povDownLeft()).whileTrue(new DistanceBasedShooting(shooter, LLVision));


    // Letters
        subjoystick.a().whileTrue(new FixedPIDShooting(shooter, 5));

        // subjoystick.b().whileTrue(new RunCommand(() -> climb.pullClimbDuty(-.2)));
        // subjoystick.b().onFalse(new RunCommand(() -> climb.stopClimb()));
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
            new RunCommand(() -> shooter.spinKickersAgain(-.4), shooter)
            )
        );
        joystick.leftTrigger().onFalse(
         
            Commands.parallel(
                new RunCommand(() -> intake.stopIntake(), intake),
                new RunCommand(() -> shooter.stopKicker(), shooter)
        ));
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
        // joystick.a().whileTrue(new LL_Orient(drivetrain, "pppr", 8, () -> -joystick.getLeftY(), () -> -joystick.getLeftX()));

        joystick.a().whileTrue(new AlignToHub(drivetrain, LLVision));
        joystick.b().whileTrue(new RunCommand(() -> this.flipDirection(1.0)));
        // joystick.x().whileTrue(new PV_Align(drivetrain, vision, VisionConstants.getMiddleTagId(), 1.5, 0, 0));
        joystick.x().whileTrue(drivetrain.applyRequest(() -> xBrake));
        joystick.y().whileTrue(new RunCommand(() -> this.flipDirection(-1.0)));


        /*
            Pathmaster implementation
        */

        joystick.rightTrigger().onTrue(pathmaster.makePathTo(Constants.ClimbConstants.getClimbPose()));
        joystick.rightTrigger().onFalse(pathmaster.cancelPathing());
        
        
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
