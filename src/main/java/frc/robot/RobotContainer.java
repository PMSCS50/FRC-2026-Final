// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.VisionConstants;
//import frc.robot.commands.ChaseTagCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import edu.wpi.first.cameraserver.CameraServer;

import frc.robot.commands.AlignToHub;
import frc.robot.commands.DistanceBasedShooting;
import frc.robot.commands.FixedPIDShooting;
import frc.robot.commands.Pivoting;
import frc.robot.commands.Intaking;
import frc.robot.pathfinding.Pathmaster;
public class RobotContainer {

    // **************************************************************************************************************
    // DRIVETRAIN CONSTANTS
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(3).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double speedLimiter = 0.5;
    private double directionFlipper = VisionConstants.getDirectionFlipper();

    //High ceiling, calculated from claude given robot config. May need to be tuned on real robot.
    private double pathMaxLinearAcceleration = 13.67; // m/s^2
    private double pathMaxAngularAcceleration = 12; // rad/s^2

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
    
    private final VisionGeneral vision;
    //private final VisionSimSystem vision;
    //private final LLSubsystem LLVision = new LLSubsystem(drivetrain, "limelight", "pppr");

    private final CommandXboxController joystick = new CommandXboxController(0);
    public static final CommandXboxController subjoystick = new CommandXboxController(1);

    private final Shooter shooter;
    private final Intake intake = new Intake();
    private final Climb climb = new Climb();
    private final Pivot pivot = new Pivot();

    public final Pathmaster monkeyDLuffy;

    /* Path follower */
    private SendableChooser<Command> autoChooser;

    double turningSpeed = 0; // for speed scaling

    public RobotContainer() {
        if (Constants.currentMode == Constants.Mode.SIM) {
            vision = new PV_Sim(drivetrain, new VisionIOSim("imaginaryPenis"));
        } else {
            vision = new LLSubsystem(drivetrain, "limelight", "pppr");
        }
        
        shooter = new Shooter(vision);
        
        monkeyDLuffy = new Pathmaster(drivetrain, MaxSpeed, pathMaxLinearAcceleration, MaxAngularRate, pathMaxAngularAcceleration);
        monkeyDLuffy.addRotationZone("TrenchBL", new Translation2d(Units.inchesToMeters(181.56-44.4), Units.inchesToMeters(0)), new Translation2d(Units.inchesToMeters(181.56+44.4), Units.inchesToMeters(49.86)), Rotation2d.kZero, true);
        monkeyDLuffy.addRotationZone("TrenchTL", new Translation2d(Units.inchesToMeters(181.56-44.4), Units.inchesToMeters(316.64-49.86)), new Translation2d(Units.inchesToMeters(181.56+44.4), Units.inchesToMeters(316.64)), Rotation2d.k180deg, true);
        monkeyDLuffy.addRotationZone("TrenchBR", new Translation2d(Units.inchesToMeters(468.56-44.4), Units.inchesToMeters(0)), new Translation2d(Units.inchesToMeters(468.56+44.4), Units.inchesToMeters(49.86)), Rotation2d.kZero, true);
        monkeyDLuffy.addRotationZone("TrenchTR", new Translation2d(Units.inchesToMeters(468.56-44.4), Units.inchesToMeters(316.64-49.86)), new Translation2d(Units.inchesToMeters(468.56+44.4), Units.inchesToMeters(316.64)), Rotation2d.k180deg, true);

        NamedCommands.registerCommand("Distance Based Shooting", new DistanceBasedShooting(shooter, vision).withTimeout(4));

        // Intaking
        NamedCommands.registerCommand("3.5 sec Intaking", new Intaking(intake).withTimeout(3.5));
        NamedCommands.registerCommand("4 sec Intaking", new Intaking(intake).withTimeout(4));
        NamedCommands.registerCommand("6 sec Intaking", new Intaking(intake).withTimeout(6));

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
        subjoystick.povDown().or(subjoystick.povDownRight()).or(subjoystick.povDownLeft()).whileTrue(new DistanceBasedShooting(shooter, vision));


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
        //joystick.rightTrigger().whileTrue(new RunCommand(() -> intake.spinIntakePID(-1), intake));
        //joystick.rightTrigger().onFalse(new RunCommand(() -> intake.stopIntake(), intake));

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
        
        if (vision instanceof LLSubsystem) {
            joystick.a().whileTrue(new AlignToHub(drivetrain, (LLSubsystem) vision));
        }
        
        //joystick.b().whileTrue(new RunCommand(() -> this.flipDirection(1.0)));
        // joystick.x().whileTrue(new PV_Align(drivetrain, vision, VisionConstants.getMiddleTagId(), 1.5, 0, 0));
        joystick.x().whileTrue(drivetrain.applyRequest(() -> xBrake));
        //joystick.y().whileTrue(new RunCommand(() -> this.flipDirection(-1.0)));


        /*
        Pathmaster implementation
        */

        /*
         * 
         */

        joystick.b().whileTrue(monkeyDLuffy.pathfindFaceTargetPose(Constants.VisionConstants.aimPose, Constants.VisionConstants.getHubPose()));
        joystick.y().onTrue(monkeyDLuffy.cancelPathing());
        
        
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
