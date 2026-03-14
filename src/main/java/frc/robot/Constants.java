// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
      // values of 50
      // public static final class L3ClimbConstants {
      //   public static final int climbMotor1CanId = 51;
      //   public static final int climbMotor2CanId = 52;
      //   public static final int slideMotor1CanId = 53;
      //   public static final int slideMotor2CanId = 54;

      //   public static final double climbSpeed = 0.78;
      //   public static final double climbMax = 30;
      //   public static final double slideSpeed = 0.6;
      // }

      public static final class VisionConstants{
        public static final Pose3d cameraToRobot = new Pose3d(0.0,0.0,0.0, new Rotation3d(0.0,0.0,0.0));
        public static final double distanceToTag = 1;
        public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

      }
      // values of 20
       public static final class IntakeConstants {
        public static final int pivotMotorCanId = 21;
        public static final int intakeMotorCanId = 22;

        public static final double pivotPower = 0.1;
        public static final double intakePower = 0.4;
      }

      // values of 30
      public static final class ShooterConstants {
        public static final int shooterMotorCanId1 = 31;
        public static final int shooterMotorCanId2 = 32;

        public static final int kickerMotorCanId1 = 33;
        public static final int kickerMotorCanId2 = 34;


        public static final double kickerMotorPower = 0.59;
        /*
        public static final int coralRoller2CanId = 22;
    
        public static final double chuteSpeed = .7;
        public static final double rollerSlowSpeed = 0.175;
        public static final double rollerFastSpeed = 0.4;
        public static final double rollerBack = -0.2;
        */
      }
      public static class hubPositionRed {
        public static final Pose2d RedHub = new Pose2d(11.912, 4.024, Rotation2d.fromDegrees(0));
      }
      public static class hubPositionBlue {
        public static final Pose2d BlueHub = new Pose2d(4.628, 4.024, Rotation2d.fromDegrees(0));
      }
     
      // values of 40
      public static final class ClimbConstants{
        public static final int climbMotorCanId = 41;
        public static final double climbSpeed = .1;
        public static final double climbMax = 15.0;
        
      }
      
      private static final Pose2d RedHub = new Pose2d(11.912, 4.024, Rotation2d.fromDegrees(0));
      private static final Pose2d BlueHub = new Pose2d(4.628, 4.024, Rotation2d.fromDegrees(0));
      public static final Pose2d HubPose = DriverStation.getAlliance() == DriverStation.Alliance.Red ? RedHub : BlueHub;
      public static final double X_REEF_ALIGNMENT_P = 2;
      public static final double Y_REEF_ALIGNMENT_P = 3;
      public static final double ROT_REEF_ALIGNMENT_P = 0.1625;
    
      public static final double ROT_SETPOINT_REEF_ALIGNMENT = -1.0;  // Rotation
      public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0.25;
      public static final double X_SETPOINT_REEF_ALIGNMENT_L4 = -0.25;  // Vertical 
      public static final double X_SETPOINT_REEF_ALIGNMENT = -0.25;
      public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.005;
      public static final double Y_SETPOINT_REEF_ALIGNMENT = 0;  // Horizontal pose
      public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.0025;
    
      public static final double DONT_SEE_TAG_WAIT_TIME = 0.5;
      public static final double POSE_VALIDATION_TIME = 1.0;
}