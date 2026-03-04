// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReefTagRelative extends Command {
    private PIDController xController, yController, rotController;
    private boolean isRightScore;
    private Timer dontSeeTagTimer, stopTimer, idleTimer;
    private CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.RobotCentric m_drive = new SwerveRequest.RobotCentric();
    private double tagID = 6;
    private boolean goL4;
    

  public AlignToReefTagRelative(CommandSwerveDrivetrain m_drivetrain, boolean isL4) {
    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.m_drivetrain = m_drivetrain;
    this.goL4 = isL4;
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    this.idleTimer = new Timer();
    this.idleTimer.start();

    rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

    if(goL4) xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT_L4);
    else xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(Constants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
        this.dontSeeTagTimer.reset();

        double[] postions = LimelightHelpers.getBotPose_TargetSpace("");

        SmartDashboard.putNumber("x", postions[2]);
        SmartDashboard.putNumber("y", postions[0]);
        SmartDashboard.putNumber("rot", postions[4]);

        double xSpeed = xController.calculate(postions[2]);
        double ySpeed = -yController.calculate(postions[0]);
        double rotValue = rotController.calculate(postions[4]);

        //drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

        m_drivetrain.setControl(m_drive
                .withVelocityX(xSpeed) // Drive forward with negative Y(forward)
                .withVelocityY(ySpeed) // Drive left with negative X (left)
                .withRotationalRate(rotValue));

        if (!rotController.atSetpoint() ||
            !yController.atSetpoint() || !xController.atSetpoint()) {
            stopTimer.reset();
        }
    }      
    else {
        m_drivetrain.setControl(m_drive
        .withVelocityX(0) // Drive forward with negative Y(forward)
        .withVelocityY(0) // Drive left with negative X (left)
        .withRotationalRate(0));
        //System.out.println("No target detected");
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setControl(m_drive
        .withVelocityX(0) // Drive forward with negative Y(forward)
        .withVelocityY(0) // Drive left with negative X (left)
        .withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return (this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
        this.stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME)) || this.idleTimer.hasElapsed(1.5);
  }
}