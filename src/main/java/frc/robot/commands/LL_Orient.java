// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LL_Orient extends Command {
    private PIDController rotController;
    private CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric();
    private double tagID = -1;
    private DoubleSupplier xInput, yInput;

    public LL_Orient(CommandSwerveDrivetrain m_drivetrain, boolean isL4, DoubleSupplier xInput, DoubleSupplier yInput) {
        rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);
        this.m_drivetrain = m_drivetrain;
        this.xInput = xInput;
        this.yInput = yInput;
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
        rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

        // Only lock onto a tag if one is currently visible
        if (LimelightHelpers.getTV("")) {
            tagID = LimelightHelpers.getFiducialID("");
        } else {
            tagID = -1;
        }
    }

    @Override
    public void execute() {
        // Confirm we still see the same tag we locked onto
        if (LimelightHelpers.getTV("") && tagID != -1 && LimelightHelpers.getFiducialID("") == tagID) {

            // [x, y, z, roll, pitch, yaw] — index 5 is yaw (rotation to face tag)
            double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
            double yaw = positions[5];

            SmartDashboard.putNumber("LL_Orient/yaw", yaw);
            SmartDashboard.putBoolean("LL_Orient/atSetpoint", rotController.atSetpoint());

            double rotValue = rotController.calculate(yaw);

            m_drivetrain.setControl(m_drive
                    .withVelocityX(xInput.getAsDouble())
                    .withVelocityY(yInput.getAsDouble())
                    .withRotationalRate(rotValue));

        } else {
            // No valid tag — stop all motion
            m_drivetrain.setControl(m_drive
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(m_drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}