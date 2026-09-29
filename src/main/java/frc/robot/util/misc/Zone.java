package frc.robot.util.misc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Represents a 2D zone on the field.
 *
 * <p>Zones can be primitive (circle, rectangle, polygon) or composite
 * (union, intersection, difference, complement).
 *
 * <p>Inspired by Team 4481's zone system.
 *
 * <p>Example:
 * <pre>
 * Zone trenchZone =
 *     new RectangleZone(
 *         new Translation2d(1, 1),
 *         new Translation2d(4, 3));
 *
 * Zone safeZone =
 *     new CircleZone(
 *         new Translation2d(5, 5),
 *         1.5);
 *
 * Zone combined = trenchZone.union(safeZone);
 *
 * combined.contains(robot::getTranslation).onTrue(hood.down());
 * </pre>
 */
public interface Zone {

  /**
   * Returns a Trigger that is active when the supplied translation is inside this zone.
   *
   * @param translation supplier providing the current translation
   * @return a trigger that polls containment
   */
  default Trigger contains(Supplier<Translation2d> translation) {
    return new Trigger(() -> contains(translation.get()));
  }

  /**
   * Returns true when the translation is inside this zone.
   *
   * @param point translation to check
   * @return whether the point is inside the zone
   */
  boolean contains(Translation2d point);

  /**
   * Returns the points defining this zone.
   *
   * <p>For primitive zones, this is an approximation or boundary representation.
   * Composite zones may not have a meaningful set of points.
   *
   * @return array of boundary points
   */
  Pose2d[] getPoints();

  /** Returns a zone representing the union (A ∪ B) of this zone and another. */
  default Zone union(Zone other) {
    return new Zone() {
      @Override
      public boolean contains(Translation2d point) {
        return Zone.this.contains(point) || other.contains(point);
      }

      @Override
      public Pose2d[] getPoints() {
        Set<Pose2d> combinedSet = new HashSet<>(Arrays.asList(Zone.this.getPoints()));
        combinedSet.addAll(Arrays.asList(other.getPoints()));
        
        return combinedSet.toArray(Pose2d[]::new); 
      }
    };
  }

  /** Returns a zone representing the intersection (A ∩ B) of this zone and another. */
  default Zone intersection(Zone other) {
    return new Zone() {
      @Override
      public boolean contains(Translation2d point) {
        return Zone.this.contains(point) && other.contains(point);
      }

      @Override
      public Pose2d[] getPoints() {
          Set<Pose2d> shared = new HashSet<>(Arrays.asList(Zone.this.getPoints()));

          shared.retainAll(Arrays.asList(other.getPoints()));

          return shared.toArray(Pose2d[]::new);
      }
    };
  }

  /**
   * Returns a zone representing the difference (A \ B):
   * points in this zone that are not in the other zone.
   */
  default Zone difference(Zone other) {
    return new Zone() {
      @Override
      public boolean contains(Translation2d point) {
        return Zone.this.contains(point) && !other.contains(point);
      }

      @Override
      public Pose2d[] getPoints() {
          Set<Pose2d> shared = new HashSet<>(Arrays.asList(Zone.this.getPoints()));

          shared.removeAll(Arrays.asList(other.getPoints()));

          return shared.toArray(Pose2d[]::new);
      }
    };
  }

  /** Returns the complement of this zone (points not in this zone). */
  default Zone complement() {
    return new Zone() {
      @Override
      public boolean contains(Translation2d point) {
        return !Zone.this.contains(point);
      }

      @Override
      public Pose2d[] getPoints() {
        return new Pose2d[0];
      }
    };
  }

  /**
   * A circular zone defined by a center point and a radius.
   */
  class CircleZone implements Zone {
    private final Translation2d center;
    private final double radius;

    public CircleZone(Translation2d center, double radius) {
      if (radius < 0) {
        throw new IllegalArgumentException("Radius cannot be negative.");
      }

      this.center = center;
      this.radius = radius;
    }

    @Override
    public boolean contains(Translation2d point) {
      return point.getDistance(center) <= radius;
    }

    @Override
    public Pose2d[] getPoints() {
      // Define how many points you want to sample along the circle
      int numPoints = 36; 
      Pose2d[] points = new Pose2d[numPoints];

      for (int i = 0; i < numPoints; i++) {
        // 1. Calculate the angle around the circle (in radians)
        double angleRad = 2 * Math.PI * i / numPoints;
        
        // 2. Calculate the X and Y coordinates relative to the center
        double x = center.getX() + radius * Math.cos(angleRad);
        double y = center.getY() + radius * Math.sin(angleRad);
        Translation2d pointLocation = new Translation2d(x, y);

        points[i] = new Pose2d(pointLocation, Rotation2d.kZero);
      }
      return points;
    }

  /**
   * An axis-aligned rectangular zone defined by two corner points.
   */
  class RectangleZone implements Zone {
    private final double minX;
    private final double maxX;
    private final double minY;
    private final double maxY;

    public RectangleZone(Translation2d cornerA, Translation2d cornerB) {
      this.minX = Math.min(cornerA.getX(), cornerB.getX());
      this.maxX = Math.max(cornerA.getX(), cornerB.getX());
      this.minY = Math.min(cornerA.getY(), cornerB.getY());
      this.maxY = Math.max(cornerA.getY(), cornerB.getY());
    }

    @Override
    public boolean contains(Translation2d point) {
      return point.getX() >= minX
          && point.getX() <= maxX
          && point.getY() >= minY
          && point.getY() <= maxY;
    }

    @Override
    public Pose2d[] getPoints() {
      return new Pose2d[] {
        new Pose2d(minX, minY, Rotation2d.kZero),
        new Pose2d(maxX, minY, Rotation2d.kZero),
        new Pose2d(maxX, maxY, Rotation2d.kZero),
        new Pose2d(minX, maxY, Rotation2d.kZero),
        new Pose2d(minX, minY, Rotation2d.kZero)
      };
    }
  }

  /**
   * A polygonal zone defined by an ordered list of vertices.
   *
   * <p>Uses a ray-casting algorithm, so both convex and concave simple
   * polygons are supported.
   */
  class PolygonZone implements Zone {
    private final List<Translation2d> vertices;
    private final Pose2d[] cachedPoints;

    /**
     * @param vertices ordered vertices of the polygon
     */
    public PolygonZone(List<Translation2d> vertices) {
      if (vertices == null || vertices.size() < 3) {
        throw new IllegalArgumentException(
            "A polygon must have at least 3 vertices.");
      }

      this.vertices = List.copyOf(vertices);

      this.cachedPoints = new Pose2d[vertices.size()];

      for (int i = 0; i < vertices.size(); i++) {
        this.cachedPoints[i] =
            new Pose2d(vertices.get(i), Rotation2d.kZero);
      }
    }

    @Override
    public boolean contains(Translation2d point) {
      return isInsidePolygon(point);
    }

    @Override
    public Pose2d[] getPoints() {
      return cachedPoints.clone();
    }

    /**
     * Ray-casting algorithm for point-in-polygon detection.
     *
     * <p>Works for both convex and concave simple polygons.
     */
    private boolean isInsidePolygon(Translation2d point) {
      int n = vertices.size();
      boolean inside = false;

      double px = point.getX();
      double py = point.getY();

      for (int i = 0, j = n - 1; i < n; j = i++) {
        double xi = vertices.get(i).getX();
        double yi = vertices.get(i).getY();

        double xj = vertices.get(j).getX();
        double yj = vertices.get(j).getY();

        boolean intersects =
            ((yi > py) != (yj > py))
                && (px < (xj - xi) * (py - yi) / (yj - yi) + xi);

        if (intersects) {
          inside = !inside;
        }
      }

      return inside;
      }
    }
  }
}