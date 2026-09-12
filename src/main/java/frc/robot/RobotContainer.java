// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.DistanceBasedShooting;
import frc.robot.commands.FixedPIDShooting;
import frc.robot.commands.FixedWaypointShooting;
import frc.robot.commands.Intaking;
import frc.robot.commands.PivotToAngle;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.pivot.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.DriveCommands;
import frc.robot.subsystems.vision.*;

import frc.robot.util.Elastic;
import frc.robot.util.ExtendedCommandXboxController;
import frc.robot.util.pathfinding.Pathmaster;
import frc.robot.util.pathfinding.commands.PostPathPreciseAlignment;
import frc.robot.util.pathfinding.commands.PostPathPreciseAlignment2;


public class RobotContainer {
    // *Drivetrain constants
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double speedLimiter = 0.7;
    //private static final double SPEED_STEP = 0.15;

    private double pathMaxLinearAcceleration = Constants.DriveConstants.pathMaxLinearAcceleration; // m/s^2
    private double pathMaxAngularAcceleration = Constants.DriveConstants.pathMaxAngularAcceleration; // rad/s^2

    private final SwerveRequest.SwerveDriveBrake xBrake = DriveConstants.xBrake;

    // *Declare and initialize subsystems and commands
    private final CommandSwerveDrivetrain drivetrain;
    private final Pathmaster monkeyDLuffy;

    private final Vision vision;
    private final Shooter shooter;
    private final Intake intake;
    private final Pivot pivot;

    public static final ExtendedCommandXboxController driverController = new ExtendedCommandXboxController(0);
    public static final ExtendedCommandXboxController operatorController = new ExtendedCommandXboxController(1);

    private static final Transform3d ROBOT_TO_CAMERA_FRONT = new Transform3d(
        new Translation3d(0.072, -0.072, 0.495),
        new Rotation3d(0, Math.toRadians(10), 0)
    );

    private static final Transform3d ROBOT_TO_CAMERA_BACK = new Transform3d(
        new Translation3d(0.072, 0.072, 0.495),
        new Rotation3d(0, Math.toRadians(10), Math.toRadians(180))
    );

    // *For choosing the auto and generating configurations for it
    private SendableChooser<Command> autoChooser;

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
        // *Initialize subsystems
        drivetrain = TunerConstants.createDrivetrain();
        monkeyDLuffy = new Pathmaster(drivetrain, MaxSpeed * speedLimiter, pathMaxLinearAcceleration, MaxAngularRate * speedLimiter, pathMaxAngularAcceleration);

        vision = new Vision(drivetrain, RobotBase.isReal() ? List.of(new VisionIOReal("", ROBOT_TO_CAMERA_FRONT)) : List.of(new VisionIOSim("imgCamFront", ROBOT_TO_CAMERA_FRONT), new VisionIOSim("imgCamBack", ROBOT_TO_CAMERA_BACK)));
        shooter = new Shooter(RobotBase.isReal() ? new ShooterIOReal() : new ShooterIOSim());
        intake = new Intake(RobotBase.isReal() ? new IntakeIOReal() : new IntakeIOSim());
        pivot = new Pivot(RobotBase.isReal() ? new PivotIOReal() : new PivotIOSim());
        
        // *Shooting
        NamedCommands.registerCommand("Fixed Based Shooting Auton", new FixedPIDShooting(shooter, 3.3).withTimeout(4));
        NamedCommands.registerCommand("Distance Based Shooting", new DistanceBasedShooting(shooter, vision, drivetrain).withTimeout(4));

        // *Intaking
        NamedCommands.registerCommand("3.5 sec Intaking", new Intaking(intake).withTimeout(3.5));
        NamedCommands.registerCommand("4 sec Intaking", new Intaking(intake).withTimeout(4));
        NamedCommands.registerCommand("6 sec Intaking", new Intaking(intake).withTimeout(6));

        // *Pivoting
        NamedCommands.registerCommand("Forward Pivoting 30%", new PivotToAngle(pivot, true).withTimeout(.5));
        NamedCommands.registerCommand("Backward Pivoting 30%" , new PivotToAngle(pivot, false).withTimeout(.5));
        NamedCommands.registerCommand("Forward Pivoting 10%", new PivotToAngle(pivot, true).withTimeout(1.5));
        NamedCommands.registerCommand("Backward Pivoting 10%" , new PivotToAngle(pivot, false).withTimeout(1.5));
        NamedCommands.registerCommand("Auton Fixed Shooting", new FixedPIDShooting(shooter, 1.366));

        // *Five shooting setpoints that form a semicircle around the hub
        for (int i = 1; i <= ShooterConstants.shootingSetpoints.length; i++) {
            monkeyDLuffy.addWaypoint(i + ":Shooting", ShooterConstants.getShootingSetpoint(i));
        }

        // *Multi-Rotation Zones (trenches)
        monkeyDLuffy.addMultiRotationZone("TrenchBL", new Translation2d(Units.inchesToMeters(181.56-44.4), Units.inchesToMeters(0)), new Translation2d(Units.inchesToMeters(181.56+44.4), Units.inchesToMeters(49.86)), List.of(Rotation2d.k180deg, Rotation2d.kZero), true);
        monkeyDLuffy.addMultiRotationZone("TrenchTL", new Translation2d(Units.inchesToMeters(181.56-44.4), Units.inchesToMeters(316.64-49.86)), new Translation2d(Units.inchesToMeters(181.56+44.4), Units.inchesToMeters(316.64)), List.of(Rotation2d.k180deg, Rotation2d.kZero), true);
        monkeyDLuffy.addMultiRotationZone("TrenchBR", new Translation2d(Units.inchesToMeters(468.56-44.4), Units.inchesToMeters(0)), new Translation2d(Units.inchesToMeters(468.56+44.4), Units.inchesToMeters(49.86)), List.of(Rotation2d.k180deg, Rotation2d.kZero), true);
        monkeyDLuffy.addMultiRotationZone("TrenchTR", new Translation2d(Units.inchesToMeters(468.56-44.4), Units.inchesToMeters(316.64-49.86)), new Translation2d(Units.inchesToMeters(468.56+44.4), Units.inchesToMeters(316.64)), List.of(Rotation2d.k180deg, Rotation2d.kZero), true);

        // *Configuring
        autoChooser = AutoBuilder.buildAutoChooser("TestingAuto");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    // *Configure Bindings
    private void configureBindings() {
        // !Driver
        // *Driving joysticks
        // drivetrain.setDefaultCommand(
        //     DriveCommands.joystickDrive(
        //         drivetrain,
        //         () -> -driverController.getLeftY() * speedLimiter,
        //         () -> -driverController.getLeftX() * speedLimiter,
        //         () -> -driverController.getRightX() * speedLimiter
        //     )
        // );
    
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                DriveCommands.joystickDriveRequest(
                    () -> -driverController.getLeftY() * speedLimiter,
                    () -> -driverController.getLeftX() * speedLimiter,
                    () -> -driverController.getRightX() * speedLimiter
                )
            )
        );

        // *Triggers and Bumpers
        driverController.leftTrigger().whileTrue(
            Commands.parallel(
                new RunCommand(() -> intake.runPID(1), intake),
                new RunCommand(() -> shooter.spinKickersSpecified(-.6), shooter)
            )
        );
        
        driverController.leftTrigger().onFalse(
            Commands.parallel(
                new RunCommand(() -> intake.stop(), intake),
                new RunCommand(() -> shooter.stopKicker(), shooter)
        ));

        driverController.leftBumper().onTrue(new InstantCommand(() -> this.setSpeed(speedLimiter - 0.15)));
        driverController.rightBumper().onTrue(new InstantCommand(() -> this.setSpeed(speedLimiter + 0.15)));

        //joystick.rightTrigger().whileTrue(new RunCommand(() -> intake.spinIntakePID(-1), intake));
        //joystick.rightTrigger().onFalse(new RunCommand(() -> intake.stopIntake(), intake));

        // *Letters
        //driverController.a().whileTrue(new LL_Orient(drivetrain, "pppr", 8, () -> -joystick.getLeftY(), () -> -joystick.getLeftX()));
        
        if (vision instanceof Vision) {
           driverController.a().whileTrue(new AlignToHub(drivetrain, vision));
        }

        driverController.b().whileTrue(
            Commands.defer(() ->
                // monkeyDLuffy.goToSelectedWaypoint()
                // .andThen(PostPathPreciseAlignment2.generateCommand(drivetrain, monkeyDLuffy.selectedWaypointPose(), Seconds.of(5.0))),
                PostPathPreciseAlignment2.generateCommand(drivetrain, monkeyDLuffy.selectedWaypointPose(), Seconds.of(5.0)),
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
                () -> intake.runPID(.2),
                () -> intake.stop(),
                intake
            )
        );

        operatorController.leftBumper().whileTrue(
            new StartEndCommand(
                () -> intake.runPID(-1),
                () -> intake.stop(),
                intake
            )
        );

        operatorController.rightTrigger().onTrue(
            new InstantCommand(
                () -> pivot.setPivotAngle(IntakeConstants.kPivotSetpointB),
                pivot
            )
        );

        operatorController.rightBumper().onTrue(
            new InstantCommand(
                () -> pivot.setPivotAngle(IntakeConstants.kPivotSetpointA),
                pivot
            )
        );

        // *POV Controls
        operatorController.povUp()
            .or(operatorController.povUpLeft())
            .or(operatorController.povUpRight())
            .whileTrue(Commands.defer(
                () -> {
                    return new FixedWaypointShooting(shooter, monkeyDLuffy.selectedWaypoint());
                }, Set.of(shooter))
                );
                               
        operatorController.povDown()
            .or(operatorController.povDownLeft())
            .or(operatorController.povDownRight())
            .whileTrue(new DistanceBasedShooting(shooter, vision, drivetrain));

        // operatorController.povLeft()
        // operatorController.povRight()

        // *Letters
        operatorController.a().whileTrue(new FixedPIDShooting(shooter, 5));
        operatorController.b().onTrue(new InstantCommand(() -> pivot.resetPivot(), pivot));
        operatorController.x().whileTrue(new RunCommand(() -> pivot.setManualDuty(.3), pivot));
        operatorController.x().onFalse(new RunCommand(() -> pivot.stop(), pivot));
        operatorController.y().whileTrue(new RunCommand(() -> pivot.setManualDuty(-.3), pivot));
        operatorController.y().onFalse(new RunCommand(() -> pivot.stop(), pivot));        
    }

    // *changing drivetrain speed: crawl, low, mid, high
    public void setSpeed(double speed) {
        speedLimiter = MathUtil.clamp(speed, 0.1, 1.0);

        if (speedLimiter <= 0.3)
            Logger.recordOutput("Drivetrain/Swerve Speed", "LOW");
        else if (speedLimiter <= 0.5)
            Logger.recordOutput("Drivetrain/Swerve Speed", "MID");
        else
            Logger.recordOutput("Drivetrain/Swerve Speed", "HIGH");
    }

    public void loadAllianceWaypoints() {
        for (int i = 1; i <= ShooterConstants.shootingSetpoints.length; i++) {
            monkeyDLuffy.addWaypoint(i + ":Shooting", ShooterConstants.getShootingSetpoint(i));
        }
    }
    

    // *Getters for subsystems and commands

    // !Run the path selected from the auto chooser
    public Command getAutonomousCommand() { 
        return autoChooser.getSelected(); 
    }

    public CommandSwerveDrivetrain getDrivetrain() { return drivetrain; }
    public Pathmaster getPathmaster() { return monkeyDLuffy; }
    public Vision getVision() { return vision; }
    public Intake getIntake() { return intake; }
    public Shooter getShooter() { return shooter; }
    public Pivot getPivot() { return pivot; }
}
