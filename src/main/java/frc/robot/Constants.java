  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.generated.TunerConstants;

// *The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants.
// !This class should not be used for any other purpose. All constants should be declared globally (i.e. public static).
// !Do not put anything functional in this class. <--- WE GOTTA FIX THIS AND PUT THE FUNCTIONS SOMEWHERE ELSE

// *It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to reduce verbosity.

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // *Enum for different robot modes, used to switch between real, sim, and replay implementations of subsystems and commands.
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // *Constants for the drivetrain
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

    // !High ceiling, calculated from claude given robot config. May need to be tuned on real robot.
    public static final double pathMaxLinearAcceleration = 13.67; // m/s^2
    public static final double pathMaxAngularAcceleration = 12; // rad/s^2
  }

  // *Constants for vision processing and AprilTag field layout
  public static class VisionConstants{
    //public static final String limelightName = "limelight-meowlit";
    public static final double HUB_ALIGN_TOLERANCE_DEG = 3.0;

    public static final int blueMiddleTagId = 26;
    public static final int redMiddleTagId = 10;

    public static final int blueLeftTagId = 21;
    public static final int redLeftTagId = 5;

    public static final int blueRightTagId = 18;
    public static final int redRightTagId = 2;

    public static int getMiddleTagId () {
      return AllianceRelativeFiducial(blueMiddleTagId);
    }
    public static int getLeftTagId () {
      return AllianceRelativeFiducial(blueLeftTagId);
    }
    public static int getRightTagId () {
      return AllianceRelativeFiducial(blueRightTagId);
    }

    public static double getDirectionFlipper() {
      return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? 1 : -1;
    }


    public static AprilTagFieldLayout aprilTagLayoutWelded = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static AprilTagFieldLayout aprilTagLayoutAndymark = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);


    public static Transform3d robotToCamera1 = new Transform3d(
      new Translation3d(0.25, -.072, 0.09),
      new Rotation3d(0, Math.toRadians(10), 0)
    );

    // *Hub Positions (IDK WHAT THIS IS DOING HERE BUT IT'S PROBABLY IMPORTANT)
    static final Pose2d RED_HUB = new Pose2d(11.901424, 4.024, Rotation2d.fromDegrees(0));
    static final Pose2d BLUE_HUB = new Pose2d(4.611624, 4.024, Rotation2d.fromDegrees(0));

    public static Pose2d getHubPose() {
      return AllianceRelativePose(BLUE_HUB);
    }

    public static Pose2d getHubPose(DriverStation.Alliance alliance) {
      return alliance == DriverStation.Alliance.Red ? RED_HUB : BLUE_HUB;
    }

    public static Pose2d getAimPose() {
      return ShooterConstants.getShootingSetpoint1();
    }

    // public static class hubPositionRed {
    //   public static final Pose2d RedHub = new Pose2d(11.912, 4.024, Rotation2d.fromDegrees(0));
    // }

    // public static class hubPositionBlue {
    //   public static final Pose2d BlueHub = new Pose2d(4.628, 4.024, Rotation2d.fromDegrees(0));
    // }

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

    
  }

  // *intake motorIDs are values of 20
  public static final class IntakeConstants {
    public static final int pivotMotorCanId = 21;
    public static final int intakeMotorCanId = 22;

    public static final double pivotPower = 0.1;
    public static final double intakePower = 0.4;

    public static final double pivotkP = 1;
    public static final double pivotkI = 0;
    public static final double pivotkD = 0;

    public static final double kMaxPivotVelocity =     1.0; // rotations per second, tune as needed
    public static final double kMaxPivotAcceleration = 1.0; // rotations per second^2, tune as needed

    public static final double kPivotSetpointA = 0.0;  
    public static final double kPivotSetpointB = 16; 
    
    public static final double pivotUpPosition   = 0/* rotations */;
    public static final double pivotDownPosition = 0.5;
    public static final double pivotTolerance    = 0.05; // rotations, tune as needed

    public static final double kPivotFreeSpeedRpm = 5676;
    public static final double kPivotStallCurrent = 30;
  }

  // *shooter motorIDs are values of 30
  public static final class ShooterConstants {
    public static final int shooterMotorCanId1 = 31;
    public static final int shooterMotorCanId2 = 32;

    public static final int kickerMotorCanId1 = 33;
    public static final int kickerMotorCanId2 = 34;


    public static final double kickerMotorPower = 0.59;
    

    // |Aimpose for testing setpoint pathing to shootinng
    // ?Facing toward blue hub at (4.611624, 4.024) from (2, 2)
    // ?Angle = atan2(2.024, 2.611624) ≈ 37.592°
    public static Pose2d shootingSetpoint1 = facePose(new Pose2d(3.03, 0.75, Rotation2d.kZero), VisionConstants.getHubPose());
    public static Pose2d shootingSetpoint2 = facePose(new Pose2d(2, 2, Rotation2d.kZero), VisionConstants.getHubPose());
    public static Pose2d shootingSetpoint3 = facePose(new Pose2d(1.6, 4, Rotation2d.kZero), VisionConstants.getHubPose());
    public static Pose2d shootingSetpoint4 = facePose(new Pose2d(2, 6, Rotation2d.kZero), VisionConstants.getHubPose());
    public static Pose2d shootingSetpoint5 = facePose(new Pose2d(3.03, 7.25, Rotation2d.kZero), VisionConstants.getHubPose());

    public static Pose2d getShootingSetpoint1() {
      return AllianceRelativePose(shootingSetpoint1);
    }

    public static Pose2d getShootingSetpoint2() {
      return AllianceRelativePose(shootingSetpoint2);
    }

    public static Pose2d getShootingSetpoint3() {
      return AllianceRelativePose(shootingSetpoint3);
    }

    public static Pose2d getShootingSetpoint4() {
      return AllianceRelativePose(shootingSetpoint4);
    }

    public static Pose2d getShootingSetpoint5() {
      return AllianceRelativePose(shootingSetpoint5);
    }
  }

  // *climb motorIDs are values of 40
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

  // *Helper methods for alliance-relative calculations
  // ?These methods take in a pose or tag ID and return the appropriate value based on the current alliance color.
  // ?This allows us to write code that is agnostic to alliance color, and just use these methods to get the correct values.
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

  // *Helper method for alliance-relative tag IDs, since the red and blue tags are in different positions on the field.
  private static int AllianceRelativeFiducial(int blueTagId) {
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
        return blueTagId - 16;
    } else {
        return blueTagId;
    }
  }

  // *Helper method to calculate the angle from the robot to a target pose, copied straight from PhotonUtils.
  private static Pose2d facePose(Pose2d pose, Pose2d facing) {
        Translation2d relativeTrl = facing.relativeTo(pose).getTranslation();
        return new Pose2d(pose.getTranslation(), new Rotation2d(relativeTrl.getX(), relativeTrl.getY()));
    }
     
  // *From Reefscape
  public static final double FIELD_MAX_X = 16.518;
  public static final double FIELD_MAX_Y = 8.043;

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

  // *Constants for tag observation and pose validation
  public static final double DONT_SEE_TAG_WAIT_TIME = 0.5;
  public static final double POSE_VALIDATION_TIME = 1.0;
}