// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.pathfinding.Pathmaster;


public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;

  public Robot() {
    Logger.recordMetadata("ProjectName", "MyProject");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();

    Pathmaster.initializePathfinder();
    m_robotContainer = new RobotContainer();

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    Pathmaster.startWarmupCommand();
  }

  //robotInit() will be removed after SystemCore, so these methods were moved to the constructor
  
  // @Override
  // public void robotInit() {
  //     DataLogManager.start();
  //     DriverStation.startDataLog(DataLogManager.getLog());

  //     Pathmaster.startWarmupCommand();
  //   }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    // if (kUseLimelight) {
    //   var driveState = m_robotContainer.drivetrain.getState();
    //   double headingDeg = driveState.Pose.getRotation().getDegrees();
    //   double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

    //   LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
    //   var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    //   if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
    //     m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
    //   }
    // }

    Logger.recordOutput("Mechanism/Shooter/shooterMotor1 subsystem rpmControl", m_robotContainer.getShooter().getVelocity());
    Logger.recordOutput("Mechanism/Shooter/shooter speed", RobotContainer.shooterSpeed);
    Logger.recordOutput("Mechanism/Pivot/pivot speed", RobotContainer.pivotSpeed);
    Logger.recordOutput("Mechanism/Pivot/pivot amount", m_robotContainer.getPivot().getPivotEncoder().getPosition());
    Logger.recordOutput("Mechanism/Intake/intake Speed", RobotContainer.intakeSpeed);
    Logger.recordOutput("Mechanism/Intake/intake motor velocity", m_robotContainer.getIntake().getIntakeEncoder().getVelocity());

    Logger.recordOutput("Vision/drivetrain/yaw", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
    Logger.recordOutput("Vision/drivetrain/x", m_robotContainer.drivetrain.getState().Pose.getX());
    Logger.recordOutput("Vision/drivetrain/y", m_robotContainer.drivetrain.getState().Pose.getY());
    Logger.recordOutput("Vision/drivetrain/distance to hub", m_robotContainer.drivetrain.getState().Pose.getTranslation().getDistance(VisionConstants.getHubPose().getTranslation()));
    Logger.recordOutput("Vision/drivetrain/pose", m_robotContainer.drivetrain.getState().Pose);

    Logger.recordOutput("Pathmaster/pathing", m_robotContainer.monkeyDLuffy.isPathing());
    Logger.recordOutput("Pathmaster/warmup", m_robotContainer.monkeyDLuffy.warmedUp());
    Logger.recordOutput("Pathmaster/AutoBuilderConfigured", m_robotContainer.monkeyDLuffy.AutoBuilderConfigured());
    Logger.recordOutput("Pathmaster/AutoBuilderPathFindingConfigured", m_robotContainer.monkeyDLuffy.AutoBuilderPathFindingConfigured());

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    //Seeds gyro to 0 if blue, 180 if red
    m_robotContainer.drivetrain.getPigeon2().setYaw((DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) ? 180.0 : 0.0);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
