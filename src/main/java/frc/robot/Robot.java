// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LLSubsystem;
import frc.robot.subsystems.Shooter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;


  private Pigeon2 mGyro = new Pigeon2(0);



  private final RobotContainer m_robotContainer;
  
  int[] redTags = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
  int[] blueTags = {17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};

  public Robot() {
    Logger.recordMetadata("ProjectName", "MyProject");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();

  
    m_robotContainer = new RobotContainer();
  }


  @Override
  public void robotInit() {
        m_robotContainer.drivetrain.getPigeon2().setYaw(DriverStation.getAlliance().get() == Alliance.Red ? 180 : 0);

      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog());
      boolean red = (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) ? true : false;
      if (red) {
        LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.limelightName, redTags);
      }
      else {
        LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.limelightName, blueTags);
      }
    }
  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("SHOOTER RPS", m_robotContainer.getShooter().getVelocity());
    


    SmartDashboard.putNumber("intake motor velocity", m_robotContainer.getIntake().getIntakeEncoder().getVelocity());

    SmartDashboard.putNumber("PIVOT AMOUNT", m_robotContainer.getPivot().getPivotEncoder().getPosition());
    SmartDashboard.putNumber("drivetrain yaw", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("drivetrain x", m_robotContainer.drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("drivetrain y", m_robotContainer.drivetrain.getState().Pose.getY());
    SmartDashboard.putNumber("drivetrain distance to hub", m_robotContainer.drivetrain.getState().Pose.getTranslation().getDistance(VisionConstants.getHubPose().getTranslation()));
    Logger.recordOutput("robotPose", m_robotContainer.drivetrain.getState().Pose);



  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    double robotYaw = mGyro.getYaw().getValueAsDouble();    
    LimelightHelpers.SetRobotOrientation(VisionConstants.limelightName, robotYaw, 0, 0, 0, 0, 0);
    LimelightHelpers.SetIMUMode(VisionConstants.limelightName,1);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.drivetrain.getPigeon2().setYaw(DriverStation.getAlliance().get() == Alliance.Red ? 180 : 0);
    LimelightHelpers.SetIMUMode(VisionConstants.limelightName, 4);


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
    LimelightHelpers.SetIMUMode(VisionConstants.limelightName, 4);

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
