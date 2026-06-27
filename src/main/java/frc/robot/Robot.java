// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.Orchestra;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.LLSubsystemMany;
import frc.robot.util.Elastic;
import frc.robot.util.pathfinding.Pathmaster;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  //private boolean allianceConfigApplied = false;
  //private String allianceColor = "Unknown";

  //Disables orchestra if false
  private boolean allowOrchestra = false;

  //Show me the power we made together!
  private final Orchestra m_orchestra = new Orchestra("audio/LR_PHY_SSJ2_Gohan_Active_Skill.chrp");

  // int[] redTags = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
  // int[] blueTags = {17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};

  private double batteryVoltage;
  private int batterytimer = 0;

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
    Pathmaster.startWarmupCommand();

    // *Add all orchestra instruments
    // |Drivetrain
    for (int i = 0; i < 4; i++) {
      m_orchestra.addInstrument(m_robotContainer.drivetrain.getModule(i).getDriveMotor(), 0);
      m_orchestra.addInstrument(m_robotContainer.drivetrain.getModule(i).getSteerMotor(), 1);
    }
    
    // |Shooter
    m_orchestra.addInstrument(m_robotContainer.getShooter().getShooterMotor1(), 0);
    m_orchestra.addInstrument(m_robotContainer.getShooter().getShooterMotor2(), 0);
  }

  // !CODE FOR ROBOT STATES
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
    //We should only be logging this once anyway
    Logger.recordOutput("RoboRIO/Brownout Voltage", RobotController.getBrownoutVoltage());

  }

  @Override
  public void robotPeriodic() {
    applyAllianceConfig();

    if (m_robotContainer.directionFlipper == 0) {
      m_robotContainer.directionFlipper = VisionConstants.getDirectionFlipper();
    }

    CommandScheduler.getInstance().run();
    m_robotContainer.monkeyDLuffy.logWaypoint();

    // |RoboRIO voltage and current monitoring
    batteryVoltage = RobotController.getBatteryVoltage();
    Logger.recordOutput("RoboRIO/Battery Voltage", batteryVoltage);

    if (batteryVoltage <= 8.5 && batterytimer >= 5000) {
      Elastic.sendNotification(
        new Elastic.Notification(
          Elastic.NotificationLevel.WARNING,
          "Battery Warning",
          "Battery voltage is low, at only " + batteryVoltage + " volts"
        )
      );
      batterytimer = 0;
    }
    // Logger.recordOutput("RoboRIO/Current 3.3V", RobotController.getCurrent3V3());
    // Logger.recordOutput("RoboRIO/Current 5V", RobotController.getCurrent5V());
    // Logger.recordOutput("RoboRIO/Current 6V", RobotController.getCurrent6V());
    // Logger.recordOutput("RoboRIO/Num Current Faults 3.3V", RobotController.getFaultCount3V3());
    // Logger.recordOutput("RoboRIO/Num Current Faults 5V", RobotController.getFaultCount5V());
    // Logger.recordOutput("RoboRIO/Num Current Faults 6V", RobotController.getFaultCount6V());
    
    // |Field (using Advantagescope for now)
    // Logger.recordOutput("Field/RobotPose", m_robotContainer.drivetrain.getPose());
    // Logger.recordOutput("Field/VisionEstimatedPose", m_robotContainer.vision.getPose());
    // Logger.recordOutput("Field/ActivePath", m_robotContainer.monkeyDLuffy.getActivePath());
    //Logger.recordOutput("Field/TargetPose", m_robotContainer.monkeyDLuffy.selectedWaypointPose());

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    
    batterytimer += 20;
  }

  // *Disabled mode
  @Override
  public void disabledInit() {
    rumbleControllers(0);
    setLimelightThrottle(0);
  }

  @Override
  public void disabledPeriodic() {
    rumbleControllers(0);

    if (Constants.currentMode == Constants.simMode) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    }

    if (!m_orchestra.isPlaying() && allowOrchestra) {
      m_orchestra.play();
    }
  }

  @Override
  public void disabledExit() {
      if (m_orchestra.isPlaying()) {
        m_orchestra.stop();
      }
  }

  // *Autonomous mode
  @Override
  public void autonomousInit() {
    Elastic.selectTab("Auto");
    applyAllianceConfig();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
    setLimelightThrottle(0);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  // *Teleop mode
  @Override
  public void teleopInit() {
    Elastic.selectTab("Teleop");
    applyAllianceConfig();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    setLimelightThrottle(0);
  }

  @Override
  public void teleopPeriodic() {
    if (m_robotContainer.vision.isAlignedToHub()) {
      rumbleControllers(0);
    } else {
      rumbleControllers(0);
    }
  }

  @Override
  public void teleopExit() {}

  // *Test mode
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    setLimelightThrottle(0);
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

  // !HELPERS
    // *Set gyro yaw based on alliance color
  private Alliance lastAppliedAlliance = null;

  private void applyAllianceConfig() {
    var allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) return;

    Alliance alliance = allianceOpt.get();
    if (alliance == lastAppliedAlliance) return;

    boolean red = alliance == Alliance.Red;
    //allianceColor = red ? "Red" : "Blue";
    m_robotContainer.drivetrain.getPigeon2().setYaw(red ? 180 : 0);
    m_robotContainer.directionFlipper = red ? 1 : -1;

    // *Clear caches first so getShootingSetpoint() recomputes with new alliance
    ShooterConstants.clearAllianceCache();

    // *Re-add waypoints with correct alliance-relative poses
    m_robotContainer.loadAllianceWaypoints();

    if (m_robotContainer.vision instanceof LLSubsystemMany ll) {
      ll.cachedHubPose = null;
      ll.hasSeededPose = false;
    }

    lastAppliedAlliance = alliance;
  }

  private void rumbleControllers(double force) {
    RobotContainer.driverController.setRumble(RumbleType.kLeftRumble, force);
    RobotContainer.driverController.setRumble(RumbleType.kRightRumble, force);
    RobotContainer.operatorController.setRumble(RumbleType.kLeftRumble, force);
    RobotContainer.operatorController.setRumble(RumbleType.kRightRumble, force);
  }

  private void setLimelightThrottle(int value) {
    NetworkTableInstance.getDefault()
        .getTable("limelight")
        .getEntry("throttle_set")
        .setNumber(value);
  }

  // // *Flip robot direction based on alliance color (if needed) for driver control
  // // |not currently used since we are using field-oriented control, but can be useful if we switch to robot-oriented control for teleop
  // private void applyAllianceDirFlip() {
  //   Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  //   boolean red = alliance == Alliance.Red;

  //   m_robotContainer.flipDirection(red ? -1 : 1); 
  // }
}