package frc.robot.util.misc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class AllianceUtil {

  public static final double FIELD_MAX_X = 16.518;
  public static final double FIELD_MAX_Y = 8.043;

  public static boolean isRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }

  public static Rotation2d getZeroRotation() {
    if (isRedAlliance()) {
      return Rotation2d.fromDegrees(180.0);
    } else {
      return Rotation2d.fromDegrees(0.0);
    }
  }

  public static Pose2d allianceRelativePose(Pose2d pose) {
    return isRedAlliance()
        ? new Pose2d(
            FIELD_MAX_X - pose.getX(),
            FIELD_MAX_Y - pose.getY(),
            pose.getRotation().rotateBy(Rotation2d.kPi))
        : pose;
  }

  public static int allianceRelativeFiducial(int blueId) {
    return isRedAlliance() ? blueId - 16 : blueId;
  }
}