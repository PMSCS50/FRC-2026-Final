// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.pathfinding.Pathmaster;


public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;

  public Robot() {
    //setUseTiming(isReal());
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

    Logger.recordOutput("RoboRIO/Battery Voltage", RobotController.getBrownoutVoltage());
    Logger.recordOutput("RoboRIO/Brownout Voltage", RobotController.getBrownoutVoltage());
    Logger.recordOutput("RoboRIO/Current 3.3V", RobotController.getCurrent3V3());
    Logger.recordOutput("RoboRIO/Current 5V", RobotController.getCurrent5V());
    Logger.recordOutput("RoboRIO/Current 6V", RobotController.getCurrent6V());
    Logger.recordOutput("RoboRIO/Num Current Faults 3.3V", RobotController.getFaultCount3V3());
    Logger.recordOutput("RoboRIO/Num Current Faults 5V", RobotController.getFaultCount5V());
    Logger.recordOutput("RoboRIO/Num Current Faults 6V", RobotController.getFaultCount6V());

    for (int i = 0; i < 4; i++) {
      SwerveModule<?, ?, ?> module = m_robotContainer.drivetrain.getModule(i);
      Logger.recordOutput("Drive/Module_" + (i+1) + "/Drivemotor/Voltage", module.getMotorVoltage().getValueAsDouble());
      Logger.recordOutput("Drive/Module_" + (i+1) + "/Drivemotor/SupplyCurrent", module.getSupplyCurrent().getValueAsDouble());
      Logger.recordOutput("Drive/Module_" + (i+1) + "/Drivemotor/StatorCurrent", module.getStatorCurrent().getValueAsDouble());
    }

    double totalDrivetrainCurrent = 0.0;
    for (int i = 0; i < 4; i++) {
        totalDrivetrainCurrent += moduleInputs[i].driveSupplyCurrentAmps;
        totalDrivetrainCurrent += moduleInputs[i].turnSupplyCurrentAmps;
    }

    Logger.recordOutput("Drive/TotalCurrent");

    Logger.recordOutput("Subsystems/Shooter/shooterMotor1 subsystem rpmControl", m_robotContainer.getShooter().getVelocity());
    Logger.recordOutput("Subsystems/Shooter/shooter speed", RobotContainer.shooterSpeed);
    Logger.recordOutput("Subsystems/Pivot/pivot speed", RobotContainer.pivotSpeed);
    Logger.recordOutput("Subsystems/Pivot/pivot amount", m_robotContainer.getPivot().getPivotEncoder().getPosition());
    Logger.recordOutput("Subsystems/Intake/intake Speed", RobotContainer.intakeSpeed);
    Logger.recordOutput("Subsystems/Intake/intake motor velocity", m_robotContainer.getIntake().getIntakeEncoder().getVelocity());

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
