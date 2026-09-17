package frc.robot.util.pathfinding.zones;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Rotation2d;

// *Utility class for helping more complex zones understand the path better
// *Helps zones like MultiRotationZone get their last target, or DynamicOrientationZone get pathpoints inbetween entry and exit
public final class ZoneUtil {
    private static List<PathPoint> points = List.of();
    private static Rotation2d startRotation = new Rotation2d();
    private static List<RotationTarget> rotationTargets = List.of();

    private ZoneUtil() {}

    public static void initialize(
        List<PathPoint> points,
        Rotation2d startRotation,
        List<RotationTarget> rotationTargets
    ) {
        ZoneUtil.points = points;
        ZoneUtil.startRotation = startRotation;
        ZoneUtil.rotationTargets = rotationTargets;
    }

    public static List<PathPoint> getPoints() {
        return points;
    }

    public static Rotation2d getStartRotation() {
        return startRotation;
    }

    public static Rotation2d getPreviousRotation(double position) {
        RotationTarget best = null;

        for (RotationTarget target : rotationTargets) {
            if (target.position() < position) {
                if (best == null || target.position() > best.position()) {
                    best = target;
                }
            }
        }

        return best != null
            ? best.rotation()
            : startRotation;
    }

    public static List<PathPoint> getPointsBetween(double entry, double exit) {
        ArrayList<PathPoint> result = new ArrayList<>();

        for (PathPoint point : points) {
            double pos = point.waypointRelativePos;

            if (pos >= entry && pos <= exit) {
                result.add(point);
            }
        }

        return result;
    }
}