package frc.robot.pathfinding;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;

public class OrientationZone extends PathZone {

    private final Pose2d targetPose;

    public OrientationZone(
            String name,
            Translation2d min,
            Translation2d max,
            Pose2d targetPose) {
        super(name, min, max);
        this.targetPose = targetPose;
    }

    @Override
    public Rotation2d getRotationAt(Translation2d pointOnPath) {
        Translation2d delta = target.getTranslation().minus(pointOnPath);
        return new Rotation2d(delta.getX(), delta.getY());
    }
}