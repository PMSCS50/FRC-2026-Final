// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.Orchestra;

import frc.robot.pathfinding.Pathmaster;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private boolean allianceConfigApplied = false;

  private boolean allowOrchestra = false; // Orchestra currently disabled.
  private final Orchestra m_choralChambersOrchestra = new Orchestra("audio/HKSS_Choral_Chambers.chrp");

  int[] redTags = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
  int[] blueTags = {17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};

  public Robot() {
    Logger.recordMetadata("ProjectName", "Orion (Pissbot)");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      Logger.addDataReceiver(new NT4Publisher());
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    Logger.start();

    Pathmaster.initializePathfinder();

    m_robotContainer = new RobotContainer();
    m_robotContainer.monkeyDLuffy.startWarmupCommand();

    for (int i = 0; i < 4; i++) {
      m_choralChambersOrchestra.addInstrument(m_robotContainer.drivetrain.getModule(i).getDriveMotor());
      m_choralChambersOrchestra.addInstrument(m_robotContainer.drivetrain.getModule(i).getSteerMotor());
    }
  }

  @Override
  public void robotInit() {
    applyAllianceConfig();
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle",     () -> m_robotContainer.drivetrain.getState().ModuleStates[0].angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity",  () -> m_robotContainer.drivetrain.getState().ModuleStates[0].speedMetersPerSecond, null);
        builder.addDoubleProperty("Front Right Angle",    () -> m_robotContainer.drivetrain.getState().ModuleStates[1].angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_robotContainer.drivetrain.getState().ModuleStates[1].speedMetersPerSecond, null);
        builder.addDoubleProperty("Back Left Angle",      () -> m_robotContainer.drivetrain.getState().ModuleStates[2].angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity",   () -> m_robotContainer.drivetrain.getState().ModuleStates[2].speedMetersPerSecond, null);
        builder.addDoubleProperty("Back Right Angle",     () -> m_robotContainer.drivetrain.getState().ModuleStates[3].angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity",  () -> m_robotContainer.drivetrain.getState().ModuleStates[3].speedMetersPerSecond, null);
        builder.addDoubleProperty("Robot Angle",          () -> m_robotContainer.drivetrain.getState().Pose.getRotation().getRadians(), null);
      }
    });
  }

  // *Set gyro yaw and Limelight fiducial filters based on alliance color
  private void applyAllianceConfig() {
      if (allianceConfigApplied) return;
      
      Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
      boolean red = alliance == Alliance.Red;

      m_robotContainer.drivetrain.getPigeon2().setYaw(red ? 180 : 0);
      m_robotContainer.vision.setFiducialIDFiltersOverrideAll(red ? redTags : blueTags);
      allianceConfigApplied = true;
  }

  // *Flip robot direction based on alliance color (if needed) for driver control
  // |not currently used since we are using field-oriented control, but can be useful if we switch to robot-oriented control for teleop
  private void applyAllianceDirFlip() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    boolean red = alliance == Alliance.Red;

    m_robotContainer.flipDirection(red ? -1 : 1); 
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.monkeyDLuffy.log();

    // |RoboRIO voltage and current monitoring
    Logger.recordOutput("RoboRIO/Battery Voltage", RobotController.getBatteryVoltage());
    Logger.recordOutput("RoboRIO/Brownout Voltage", RobotController.getBrownoutVoltage());
    Logger.recordOutput("RoboRIO/Current 3.3V", RobotController.getCurrent3V3());
    Logger.recordOutput("RoboRIO/Current 5V", RobotController.getCurrent5V());
    Logger.recordOutput("RoboRIO/Current 6V", RobotController.getCurrent6V());
    Logger.recordOutput("RoboRIO/Num Current Faults 3.3V", RobotController.getFaultCount3V3());
    Logger.recordOutput("RoboRIO/Num Current Faults 5V", RobotController.getFaultCount5V());
    Logger.recordOutput("RoboRIO/Num Current Faults 6V", RobotController.getFaultCount6V());

    
    Logger.recordOutput("Field/RobotPose", m_robotContainer.drivetrain.getPose());
    Logger.recordOutput("Field/VisionEstimatedPose", m_robotContainer.vision.getPose());
    Logger.recordOutput("Field/ActivePath", m_robotContainer.monkeyDLuffy.getActivePath());
    Logger.recordOutput("Field/TargetPose", m_robotContainer.monkeyDLuffy.selectedWaypointPose());

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    if (allowOrchestra){
      if (!m_choralChambersOrchestra.isPlaying() && m_robotContainer.drivetrain.getSpeeds().equals(new ChassisSpeeds())) {
      m_choralChambersOrchestra.play();

      } else if (m_choralChambersOrchestra.isPlaying() && !m_robotContainer.drivetrain.getSpeeds().equals(new ChassisSpeeds())) {
        m_choralChambersOrchestra.stop();
      }
    }
    
  }

  // *Disabled mode
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    //DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

    double robotYaw = m_robotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble();
    // m_robotContainer.vision.setRobotOrientationAll(robotYaw, 0, 0, 0, 0, 0);
    //m_robotContainer.vision.setIMUModeAll(1);
  }

  @Override
  public void disabledExit() {}

  // *Autonomous mode
  @Override
  public void autonomousInit() {
    applyAllianceConfig();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  // *Teleop mode
  @Override
  public void teleopInit() {

    //applyAllianceDirFlip();
    applyAllianceConfig();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  // *Test mode
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  // *Simulation mode
  @Override
  public void simulationInit() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
  }

  @Override
  public void simulationPeriodic() {
    m_robotContainer.drivetrain.updateSimState(0.02, RobotController.getBatteryVoltage());
  }
}