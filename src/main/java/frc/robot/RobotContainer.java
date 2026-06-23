// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.AlignToHub;
import frc.robot.commands.DistanceBasedShooting;
import frc.robot.commands.FixedPIDShooting;
import frc.robot.commands.FixedWaypointShooting;
import frc.robot.commands.Pivoting;
import frc.robot.commands.Intaking;
import frc.robot.commands.PostPathPreciseAlignment;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.vision.*;
import frc.robot.util.Elastic;
import frc.robot.util.pathfinding.Pathmaster;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

import frc.robot.generated.TunerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class RobotContainer {
    // *DRIVETRAIN CONSTANTS
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double speedLimiter = 0.5;
    public double directionFlipper;

    private double pathMaxLinearAcceleration = Constants.DriveConstants.pathMaxLinearAcceleration; // m/s^2
    private double pathMaxAngularAcceleration = Constants.DriveConstants.pathMaxAngularAcceleration; // rad/s^2

    // *EXTRA SETUP - I GOT NO CLUE
    // *Setting up bindings for necessary control of the swerve drive platform
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 2% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake xBrake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    //         .withDriveRequestType(DriveRequestType.Velocity);
    

    //private final Telemetry logger = new Telemetry(MaxSpeed);

    //! ACTUAL IMPORTANT STUFF (initiallize subsystems and the like)
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final VisionGeneral vision;
    public final Pathmaster monkeyDLuffy;
    private final Shooter shooter;
    private final Intake intake = new Intake();
    private final Pivot pivot = new Pivot();

    public static final CommandXboxController driverController = new CommandXboxController(0);
    public static final CommandXboxController operatorController = new CommandXboxController(1);

    // Path follower
    private SendableChooser<Command> autoChooser;

    // |we arent even using ts, ill decide if we should remove it when we test next next week
    public double turningSpeed = 0; // for speed scaling

    public static RobotConfig robotConfig = null;
    static {
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            Elastic.sendNotification(
                new Elastic.Notification().
                withLevel(Elastic.NotificationLevel.ERROR)
                .withTitle("RobotConfig Not initialized")
                .withDescription("Could not properly load RobotConfig"));
        }
    }
    

    // *Constructor
    public RobotContainer() {
        if (Constants.currentMode == Constants.Mode.SIM) {
            vision = new PV_Sim(drivetrain, new VisionIOSim("imaginaryPenis"));
        } else {
            vision = new LLSubsystemMany(drivetrain, "");
        }
        
        shooter = new Shooter(vision);
        monkeyDLuffy = new Pathmaster(drivetrain, MaxSpeed * 0.15, pathMaxLinearAcceleration, MaxAngularRate * 0.15, pathMaxAngularAcceleration);
        // Populate default alliance waypoints immediately so selectedWaypoint() is safe
        loadAllianceWaypoints();
        
        // *Shooting
        NamedCommands.registerCommand("Fixed Based Shooting Auton", new FixedPIDShooting(shooter, 3.3).withTimeout(4));
        NamedCommands.registerCommand("Distance Based Shooting", new DistanceBasedShooting(shooter, vision).withTimeout(4));
    // Additional named shooting variants used by autos
    NamedCommands.registerCommand("T-26 Distance Based Shooting", new DistanceBasedShooting(shooter, vision).withTimeout(4));
    NamedCommands.registerCommand("T-2 Distance Based Shooting", new DistanceBasedShooting(shooter, vision).withTimeout(4));
    NamedCommands.registerCommand("4 sec Middle Distance Based Shooting", new DistanceBasedShooting(shooter, vision).withTimeout(4));
    NamedCommands.registerCommand("Fixed Shooting Left Shoot", new FixedWaypointShooting(shooter, monkeyDLuffy.selectedWaypoint()));

        // *Intaking
        NamedCommands.registerCommand("3.5 sec Intaking", new Intaking(intake).withTimeout(3.5));
        NamedCommands.registerCommand("4 sec Intaking", new Intaking(intake).withTimeout(4));
        NamedCommands.registerCommand("6 sec Intaking", new Intaking(intake).withTimeout(6));

        // *Pivoting
        NamedCommands.registerCommand("Forward Pivoting 30%", new Pivoting(pivot, true).withTimeout(.5));
        NamedCommands.registerCommand("Pivoting Back 30%" , new Pivoting(pivot, false).withTimeout(.5));
        NamedCommands.registerCommand("Forward Pivoting 10%", new Pivoting(pivot, true).withTimeout(1.5));
        NamedCommands.registerCommand("Pivoting Back 10%" , new Pivoting(pivot, false).withTimeout(1.5));
    // Aliases used in path files
    NamedCommands.registerCommand("Forward Pivoting", new Pivoting(pivot, true));
    NamedCommands.registerCommand("Backward Pivoting 10%", new Pivoting(pivot, false).withTimeout(1.5));

        // *Five shooting setpoints that form a semicircle around the hub
        // for (int i = 1; i <= ShooterConstants.shootingSetpoints.length; i++) {
        //     monkeyDLuffy.addWaypoint(i + ":Shooting", ShooterConstants.getShootingSetpoint(i));
        // }

        // *Rotation Zones (trenches)
        monkeyDLuffy.addRotationZone("TrenchBL", new Translation2d(Units.inchesToMeters(181.56-44.4), Units.inchesToMeters(0)), new Translation2d(Units.inchesToMeters(181.56+44.4), Units.inchesToMeters(49.86)), Rotation2d.kZero, true);
        monkeyDLuffy.addRotationZone("TrenchTL", new Translation2d(Units.inchesToMeters(181.56-44.4), Units.inchesToMeters(316.64-49.86)), new Translation2d(Units.inchesToMeters(181.56+44.4), Units.inchesToMeters(316.64)), Rotation2d.k180deg, true);
        monkeyDLuffy.addRotationZone("TrenchBR", new Translation2d(Units.inchesToMeters(468.56-44.4), Units.inchesToMeters(0)), new Translation2d(Units.inchesToMeters(468.56+44.4), Units.inchesToMeters(49.86)), Rotation2d.kZero, true);
        monkeyDLuffy.addRotationZone("TrenchTR", new Translation2d(Units.inchesToMeters(468.56-44.4), Units.inchesToMeters(316.64-49.86)), new Translation2d(Units.inchesToMeters(468.56+44.4), Units.inchesToMeters(316.64)), Rotation2d.k180deg, true);

        // *Configuring
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    // *Configure Bindings
    private void configureBindings() {
        // !Driver
        // *Driving joysticks
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double forward = driverController.getLeftY() * MaxSpeed * directionFlipper * speedLimiter;
                double translation = driverController.getLeftX() * MaxSpeed * directionFlipper * speedLimiter;
                double turn = driverController.getRightX() * MaxAngularRate * speedLimiter* -1;
                return drive
                    .withVelocityX(forward)
                    .withVelocityY(translation)
                    .withRotationalRate(turn);
            })
        );

        // *Triggers and Bumpers
       driverController.leftTrigger().whileTrue(
            Commands.parallel(
                new RunCommand(() -> intake.spinIntakePID(1), intake),
                new RunCommand(() -> shooter.spinKickersSpecified(-.6), shooter)
            )
        );
        
       driverController.leftTrigger().onFalse(
            Commands.parallel(
                new RunCommand(() -> intake.stopIntake(), intake),
                new RunCommand(() -> shooter.stopKicker(), shooter)
        ));

       driverController.leftBumper().onTrue(new InstantCommand(() -> this.setSpeed(speedLimiter-.1)));

       driverController.rightBumper().onTrue(new InstantCommand(() -> this.setSpeed(speedLimiter+.1)));
       driverController.rightTrigger().and(driverController.povDownLeft()).onTrue(new InstantCommand(() -> this.flipDirection()));

        //joystick.rightTrigger().whileTrue(new RunCommand(() -> intake.spinIntakePID(-1), intake));
        //joystick.rightTrigger().onFalse(new RunCommand(() -> intake.stopIntake(), intake));

        // *Letters
        //driverController.a().whileTrue(new LL_Orient(drivetrain, "pppr", 8, () -> -joystick.getLeftY(), () -> -joystick.getLeftX()));
        
        if (vision instanceof LLSubsystemMany) {
           driverController.a().whileTrue(new AlignToHub(drivetrain, (LLSubsystemMany) vision));
        }

       driverController.b().whileTrue(
            Commands.defer(
                () -> monkeyDLuffy.goToSelectedWaypoint()
                    .andThen(new PostPathPreciseAlignment(drivetrain, monkeyDLuffy.selectedWaypointPose(), robotConfig)),
                Set.of(drivetrain)
            )
        );

       driverController.x().whileTrue(drivetrain.applyRequest(() -> xBrake));
       driverController.y().whileTrue(new InstantCommand(() -> monkeyDLuffy.selectNextWaypoint()));

        // *POV Controls
        //driverController.povUp()
        //driverController.povRight()
        //driverController.povLeft()
        //driverController.povUp()

        //driverController.povUp().whileTrue(new RunCommand(() -> this.setSpeed(1.0)));
        //driverController.povRight().whileTrue(new RunCommand(() -> this.setSpeed(0.500)));
        //driverController.povLeft().whileTrue(new RunCommand(() -> this.setSpeed(0.200)));
        //driverController.povDown().whileTrue(new RunCommand(() -> this.setSpeed(0.1)));

        //! Operator
        // *Triggers and Bumpers
        // operatorController.leftTrigger().whileTrue(new RunCommand(() -> intake.spinIntakePID(1), intake));
        // operatorController.leftBumper().and(operatorController.leftTrigger().negate())
        //     .whileTrue(new RunCommand(() -> intake.spinIntakePID(-1), intake));
        // operatorController.leftBumper().and(operatorController.leftTrigger())
        //     .onFalse(new RunCommand(() -> intake.stopIntake(), intake));

        operatorController.leftTrigger().whileTrue(
            new StartEndCommand(
                () -> intake.spinIntakePID(1),
                () -> intake.stopIntake(),
                intake
            )
        );
        operatorController.leftBumper().whileTrue(
            new StartEndCommand(
                () -> intake.spinIntakePID(-1),
                () -> intake.stopIntake(),
                intake
            )
        );

        operatorController.rightTrigger().onTrue(
            new InstantCommand(
                () -> pivot.goToPositionMAXMotion(IntakeConstants.kPivotSetpointB, ClosedLoopSlot.kSlot0),
                pivot
            )
        );

        operatorController.rightBumper().onTrue(
            new InstantCommand(
                () -> pivot.goToPositionMAXMotion(IntakeConstants.kPivotSetpointA, ClosedLoopSlot.kSlot1),
                pivot
            )
        );

        // *POV Controls
        operatorController.povUp().or(operatorController.povUpLeft()).or(operatorController.povUpRight()).whileTrue(
            Commands.defer(() -> new FixedWaypointShooting(shooter, monkeyDLuffy.selectedWaypoint()), Set.of(shooter)));
        operatorController.povDown().or(operatorController.povDownLeft()).or(operatorController.povDownRight()).whileTrue(new DistanceBasedShooting(shooter,vision));

        // operatorController.povLeft()
        // operatorController.povRight()

        // *Letters
        operatorController.a().whileTrue(new FixedPIDShooting(shooter, 5));
        operatorController.b().onTrue(new InstantCommand(() -> pivot.resetPivot(), pivot));
        operatorController.x().whileTrue(new RunCommand(() -> pivot.spinPivotDuty(.1), pivot));
        operatorController.x().onFalse(new RunCommand(() -> pivot.stopPivot(), pivot));
        operatorController.y().whileTrue(new RunCommand(() -> pivot.spinPivotDuty(-.1), pivot));
        operatorController.y().onFalse(new RunCommand(() -> pivot.stopPivot(), pivot));        
    }

    // *changing drivetrain speed: crawl, low, mid, high
    public void setSpeed(double speed) {
        if (speed < 0.1) {
            speed = 0.1;
            Logger.recordOutput("Drivetrain/Swerve Speed", "CRAWL");
        }
        if(speed <= .25) {
            turningSpeed = .25;
            Logger.recordOutput("Drivetrain/Swerve Speed", "LOW");
        } else {
            turningSpeed =driverController.getRightX() * MaxAngularRate * speedLimiter * directionFlipper;

            if (speed <= 0.3) Logger.recordOutput("Drivetrain/Swerve Speed", "LOW");
            else if (speed <= 0.5) Logger.recordOutput("Drivetrain/Swerve Speed", "MID");
            else Logger.recordOutput("Drivetrain/Swerve Speed", "HIGH");
        }
        speedLimiter = speed;
        
    }

    // *flipping direction for driver orientation
    public void flipDirection(){
        directionFlipper = directionFlipper * -1;
    }

    public void loadAllianceWaypoints() {
    for (int i = 1; i <= ShooterConstants.shootingSetpoints.length; i++) {
        monkeyDLuffy.addWaypoint(i + ":Shooting", ShooterConstants.getShootingSetpoint(i));
    }
}
    

    // *Getters for subsystems and commands

    // !Run the path selected from the auto chooser
    public Command getAutonomousCommand() { return autoChooser.getSelected(); }

    public Intake getIntake() { return intake; }
    public Shooter getShooter() { return shooter; }
    public Pivot getPivot() { return pivot; }
}
