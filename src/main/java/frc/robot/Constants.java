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

      public static class VisionConstants{
        
        public static Pose3d cameraToRobot = new Pose3d(0.0,0.0,0.0, new Rotation3d(0.0,0.0,0.0));
        public static double distanceToTag = 1;
        public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        public static String camera0Name = "camera0_2585";
        public static String camera1Name = "camera1_2585";

        public static Transform3d robotToCamera1 = new Transform3d(
          new Translation3d(0.25, -.072, 0.09),
          new Rotation3d(0, Math.toRadians(10), 0)
        );

        // Hub Positions
        private static final Pose2d RedHub = new Pose2d(11.912, 4.024, Rotation2d.fromDegrees(0));
        private static final Pose2d BlueHub = new Pose2d(4.628, 4.024, Rotation2d.fromDegrees(0));
        public static Pose2d getHubPose() {
          return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? RedHub : BlueHub;
        }
        // Basic filtering thresholds
        public static double maxAmbiguity = 0.3;
        public static double maxZError = 0.75;

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        public static double linearStdDevBaseline = 0.02; // Meters
        public static double angularStdDevBaseline = 0.06; // Radians

        // Standard deviation multipliers for each camera
        // (Adjust to trust some cameras more than others)
        public static double[] cameraStdDevFactors =
          new double[] {
            1.0, // Camera 0
            1.0 // Camera 1
          };

        // Multipliers to apply for MegaTag 2 observations
        public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
        public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available

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