  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

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
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }
        public static class DriveConstants {

          public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
          public static final double MaxAngularRate = RotationsPerSecond.of(3).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


          public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 2% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
          public static final SwerveRequest.SwerveDriveBrake xBrake = new SwerveRequest.SwerveDriveBrake();
          public static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
          public static final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                  .withDriveRequestType(DriveRequestType.Velocity);

          //High ceiling, calculated from claude given robot config. May need to be tuned on real robot.
          public static final double pathMaxLinearAcceleration = 13.67; // m/s^2
          public static final double pathMaxAngularAcceleration = 12; // rad/s^2

        }



        public static class VisionConstants{
          public static final String limelightName = "limelight-meowlit";

          public static final int blueMiddleTagId = 26;
          public static final int redMiddleTagId = 10;

          public static final int blueLeftTagId = 21;
          public static final int redLeftTagId = 5;

          public static final int blueRightTagId = 18;
          public static final int redRightTagId = 2;

          public static int getMiddleTagId () {
            return AllianceRelativeFiducials(blueMiddleTagId, redMiddleTagId);
          }
          public static int getLeftTagId () {
            return AllianceRelativeFiducials(blueLeftTagId, redLeftTagId);
          }
          public static int getRightTagId () {
            return AllianceRelativeFiducials(blueRightTagId, redRightTagId);
          }

          public static int getDirectionFlipper() {
            return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? 1 : -1;
          }

          // public static Pose3d cameraToRobot = new Pose3d(0.0,0.0,0.0, new Rotation3d(0.0,0.0,0.0));
          // public static double distanceToTag = 1;
          public static AprilTagFieldLayout aprilTagLayoutWelded = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
          public static AprilTagFieldLayout aprilTagLayoutAndymark = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);


          public static Transform3d robotToCamera1 = new Transform3d(
            new Translation3d(0.25, -.072, 0.09),
            new Rotation3d(0, Math.toRadians(10), 0)
          );

          // Hub Positions (IDK WHAT THIS IS DOING HERE BUT IT'S PROBABLY IMPORTANT)
          static final Pose2d RedHub = new Pose2d(11.901424, 4.024, Rotation2d.fromDegrees(0));
          static final Pose2d BlueHub = new Pose2d(4.611624, 4.024, Rotation2d.fromDegrees(0));

          private static final Pose2d RedHubRedSide = new Pose2d(4.611624, 4.024, Rotation2d.fromDegrees(0));
          private static final Pose2d RedHubBlueSide = new Pose2d(11.901424, 4.024, Rotation2d.fromDegrees(0));
          
          private static final Pose2d BlueHubRedSide = new Pose2d(11.901424, 4.024, Rotation2d.fromDegrees(0));
          private static final Pose2d BlueHubBlueSide = new Pose2d(4.611624, 4.024, Rotation2d.fromDegrees(0));

          public static Pose2d getBlueHubPose() {
            return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? BlueHubRedSide : BlueHubBlueSide;
          }
           public static Pose2d getRedHubPose() {
            return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? RedHubRedSide : RedHubBlueSide;
          }
           public static Pose2d getHubPose2() {
            return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? getRedHubPose() : getBlueHubPose();
          }


          public static Pose2d getHubPose() {
            return AllianceRelativePose(BlueHub);
          }

          public static Pose2d getHubPose(DriverStation.Alliance alliance) {
            return alliance == DriverStation.Alliance.Red ? RedHub : BlueHub;
          }

          public static LoggedNetworkNumber centerX = new LoggedNetworkNumber("Vision/Center/X", 8.270494);
          public static LoggedNetworkNumber centerY = new LoggedNetworkNumber("Vision/Center/Y", 4.034536);
          public static LoggedNetworkNumber centerRot = new LoggedNetworkNumber("Vision/Center/Rot", 0.0);

          public static Pose2d getCenter() {
              return new Pose2d(
                  centerX.get(),
                  centerY.get(),
                  Rotation2d.fromDegrees(centerRot.get())
              );
          }

          // Aimpose for testing set-point pathing to shooting
          // Facing toward blue hub at (4.611624, 4.024) from (2, 2)
          // Angle = atan2(2.024, 2.611624) ≈ 37.592°
          public static Pose2d aimPose = AllianceRelativePose(new Pose2d(2, 2, Rotation2d.fromDegrees(37.592)));
          // Basic filtering thresholds
          // public static double maxAmbiguity = 0.3;
          // public static double maxZError = 0.75;

          // // Standard deviation baselines, for 1 meter distance and 1 tag
          // // (Adjusted automatically based on distance and # of tags)
          // public static double linearStdDevBaseline = 0.02; // Meters
          // public static double angularStdDevBaseline = 0.06; // Radians

          // // Standard deviation multipliers for each camera
          // // (Adjust to trust some cameras more than others)
          // public static double[] cameraStdDevFactors =
          //   new double[] {
          //     1.0, // Camera 0
          //     1.0 // Camera 1
          //   };

          // // Multipliers to apply for MegaTag 2 observations
          // public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
          // public static double angularStdDevMegatag2Factor =
          // Double.POSITIVE_INFINITY; // No rotation data available

        }
        // values of 20
        public static final class IntakeConstants {
          public static final int pivotMotorCanId = 21;
          public static final int intakeMotorCanId = 22;

          public static final double pivotPower = 0.1;
          public static final double intakePower = 0.4;

          // public static final double kP = 0;
          // public static final double kI = 0;
          // public static final double kD = 0;

          public static final double kPivotSetpointA = 0.0;  
          public static final double kPivotSetpointB = 16; 
          
          public static final double pivotUpPosition   = 0/* rotations */;
          public static final double pivotDownPosition = 0.5;
          public static final double pivotTolerance    = 0.05; // rotations, tune as needed
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
          private static final Pose2d RedClimb = new Pose2d(14.94, 3.71, Rotation2d.fromDegrees(0));
          private static final Pose2d BlueClimb = new Pose2d(1.6, 3.71, Rotation2d.fromDegrees(0));
          public static Pose2d getClimbPose() {
            return AllianceRelativePose(BlueClimb);
          }

          public static Pose2d getClimbPose(DriverStation.Alliance alliance) {
            return alliance == DriverStation.Alliance.Red ? RedClimb : BlueClimb;
          }
          
        }

        private static Pose2d AllianceRelativePose(Pose2d pose) {
          if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
              return new Pose2d(
                  FIELD_MAX_X - pose.getX(),
                  FIELD_MAX_Y - pose.getY(),
                  pose.getRotation().plus(Rotation2d.fromDegrees(180))
              );
          } else {
              return pose;
          }
        }

        private static int AllianceRelativeFiducials(int blueTagId, int redTagId) {
          if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
              return redTagId;
          } else {
              return blueTagId;
          }
        }
        
        public static final double FIELD_MAX_X = 16.513048;
        public static final double FIELD_MAX_Y = 8.042656;

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