// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Constants.VisionConstants;
import frc.robot.pathfinding.Pathmaster;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  //private Pigeon2 mGyro = new Pigeon2(0);
  private final RobotContainer m_robotContainer;
  private boolean allianceConfigApplied = false;

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

    // DataLogManager.start();
    // DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotInit() {
    applyAllianceConfig();
  }

  // *Set gyro yaw and Limelight fiducial filters based on alliance color
  private void applyAllianceConfig() {
      if (allianceConfigApplied) return;
      
      Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
      boolean red = alliance == Alliance.Red;

      m_robotContainer.drivetrain.getPigeon2().setYaw(red ? 180 : 0);
      m_robotContainer.vision.SetFiducialIDFiltersOverride( VisionConstants.limelightName, red ? redTags : blueTags);
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

    // *Logging and SmartDashboard updates
    // |Drivetrain pose and heading
    Logger.recordOutput("Drivetrain/Yaw", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
    Logger.recordOutput("Drivetrain/X", m_robotContainer.drivetrain.getState().Pose.getX());
    Logger.recordOutput("Drivetrain/Y", m_robotContainer.drivetrain.getState().Pose.getY());
    Logger.recordOutput("Drivetrain/Distance to Hub", m_robotContainer.drivetrain.getState().Pose.getTranslation().getDistance(VisionConstants.getHubPose().getTranslation()));
    Logger.recordOutput("Drivetrain/Robot Pose", m_robotContainer.drivetrain.getState().Pose);

    // |RoboRIO voltage and current monitoring
    Logger.recordOutput("RoboRIO/Battery Voltage", RobotController.getBatteryVoltage());
    Logger.recordOutput("RoboRIO/Brownout Voltage", RobotController.getBrownoutVoltage());
    Logger.recordOutput("RoboRIO/Current 3.3V", RobotController.getCurrent3V3());
    Logger.recordOutput("RoboRIO/Current 5V", RobotController.getCurrent5V());
    Logger.recordOutput("RoboRIO/Current 6V", RobotController.getCurrent6V());
    Logger.recordOutput("RoboRIO/Num Current Faults 3.3V", RobotController.getFaultCount3V3());
    Logger.recordOutput("RoboRIO/Num Current Faults 5V", RobotController.getFaultCount5V());
    Logger.recordOutput("RoboRIO/Num Current Faults 6V", RobotController.getFaultCount6V());

    // |Swerve module states and motor outputs
    for (int i = 0; i < 4; i++) {
      SwerveModule<?, ?, ?> module = m_robotContainer.drivetrain.getModule(i);
      Logger.recordOutput("Drive/Module_" + (i+1) + "/Drivemotor/Voltage", module.getDriveMotor().getMotorVoltage().getValueAsDouble());
      Logger.recordOutput("Drive/Module_" + (i+1) + "/Drivemotor/SupplyCurrent", module.getDriveMotor().getSupplyCurrent().getValueAsDouble());
      Logger.recordOutput("Drive/Module_" + (i+1) + "/Drivemotor/StatorCurrent", module.getDriveMotor().getStatorCurrent().getValueAsDouble());

      Logger.recordOutput("Drive/Module_" + (i+1) + "/Turnmotor/Voltage", module.getSteerMotor().getMotorVoltage().getValueAsDouble());
      Logger.recordOutput("Drive/Module_" + (i+1) + "/Turnmotor/SupplyCurrent", module.getSteerMotor().getSupplyCurrent().getValueAsDouble());
      Logger.recordOutput("Drive/Module_" + (i+1) + "/Turnmotor/StatorCurrent", module.getSteerMotor().getStatorCurrent().getValueAsDouble());
    }

    // |Subsystems
    Logger.recordOutput("Subsystems/Shooter/Shooter shooterMotor1 Angular Velocity", m_robotContainer.getShooter().getVelocity());
    Logger.recordOutput("Subsystems/Shooter/Shooter Speed", RobotContainer.getShooterSpeed());

    Logger.recordOutput("Subsystems/Pivot/Pivot Speed", RobotContainer.getPivotSpeed());
    Logger.recordOutput("Subsystems/Pivot/Pivot Amount", m_robotContainer.getPivot().getPivotEncoder().getPosition());
    Logger.recordOutput("Subsystems/Pivot/Pivot Motor Voltage", m_robotContainer.getPivot().getPivotMotor().getBusVoltage()); // sparkmax

    Logger.recordOutput("Subsystems/Intake/Intake Speed", RobotContainer.getIntakeSpeed());
    Logger.recordOutput("Subsystems/Intake/Intake Motor Velocity", m_robotContainer.getIntake().getIntakeEncoder().getVelocity());
    //Logger.recordOutput("Subsystems/Intake/Intake Motor Voltage", m_robotContainer.getIntake().getIntakeMotor().getBusVoltage()); // sparkmax

    // |Pathmaster
    Logger.recordOutput("Pathmaster/pathing", m_robotContainer.monkeyDLuffy.isPathing());
    Logger.recordOutput("Pathmaster/warmup", m_robotContainer.monkeyDLuffy.warmedUp());
    Logger.recordOutput("Pathmaster/AutoBuilderConfigured", m_robotContainer.monkeyDLuffy.AutoBuilderConfigured());
    Logger.recordOutput("Pathmaster/AutoBuilderPathFindingConfigured", m_robotContainer.monkeyDLuffy.AutoBuilderPathFindingConfigured());
    Logger.recordOutput("Pathmaster/Selected Waypoint", m_robotContainer.monkeyDLuffy.selectedWaypoint());
    Logger.recordOutput("Pathmaster/Selected Waypoint Pose", m_robotContainer.monkeyDLuffy.selectedWaypointPose());
  }

  // *Disabled mode
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    double robotYaw = m_robotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble();
    //LimelightHelpers.SetRobotOrientation(VisionConstants.limelightName, robotYaw, 0, 0, 0, 0, 0);
    m_robotContainer.vision.setRobotOrientationAll(robotYaw, 0, 0, 0, 0, 0);
    m_robotContainer.vision.setIMUModeAll(1);
  }

  @Override
  public void disabledExit() {}

  // *Autonomous mode
  @Override
  public void autonomousInit() {
    applyAllianceConfig();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  // *Teleop mode
  @Override
  public void teleopInit() {
    LimelightHelpers.SetIMUMode(VisionConstants.limelightName, 4);

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
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    m_robotContainer.drivetrain.updateSimState(0.02, RobotController.getBatteryVoltage());
  }
}