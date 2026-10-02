package frc.robot.util.misc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
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
        return GeometryUtil.union(Zone.this.getPoints(), other.getPoints());
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
          return GeometryUtil.intersection(Zone.this.getPoints(), other.getPoints());
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
          return GeometryUtil.difference(Zone.this.getPoints(), other.getPoints());
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
        return Zone.this.getPoints(); // Points are inverted, but the boundary representation remains the same.
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

  class EllipseZone implements Zone {
    private final Translation2d center;
    private final double xAxis;
    private final double yAxis;
    private final Rotation2d rotation;

    public EllipseZone(
            Translation2d center,
            double xAxis,
            double yAxis,
            Rotation2d rotation) {
        if (xAxis <= 0 || yAxis <= 0) {
            throw new IllegalArgumentException("Axes must be positive.");
        }

        this.center = center;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.rotation = rotation;
    }

    public EllipseZone(
            Translation2d center,
            double xAxis,
            double yAxis) {
      this(center, xAxis, yAxis, Rotation2d.kZero);
    }

    //If you are confused... Then I cant help you idk wtf is going on
    //This sdEllipse algorithm was copied from https://iquilezles.org/articles/distfunctions2d/#:~:text=float-,sdEllipse,-(%20in%20vec2
    private double sdEllipse(Translation2d point) {
      Translation2d local = point
              .minus(center)
              .rotateBy(rotation);

      double px = Math.abs(local.getX());
      double py = Math.abs(local.getY());

      double ax = xAxis;
      double ay = yAxis;

      if (px > py) {
          double temp = px;
          px = py;
          py = temp;

          temp = ax;
          ax = ay;
          ay = temp;
      }

      if (Math.abs(ax - ay) < 1e-12) {
          return Math.hypot(px, py) - ax;
      }

      double l = ay * ay - ax * ax;
      double m = ax * px / l;
      double m2 = m * m;
      double n = ay * py / l;
      double n2 = n * n;
      double c = (m2 + n2 - 1.0) / 3.0;
      double c3 = c * c * c;
      double q = c3 + m2 * n2 * 2.0;
      double d = c3 + m2 * n2;
      double g = m + m * n2;

      double co;

      if (d < 0.0) {
          double h = Math.acos(q / c3) / 3.0;
          double s = Math.cos(h);
          double t = Math.sin(h) * Math.sqrt(3.0);
          double rx = Math.sqrt(-c * (s + t + 2.0) + m2);
          double ry = Math.sqrt(-c * (s - t + 2.0) + m2);

          co = (
                  ry
                  + Math.signum(l) * rx
                  + Math.abs(g) / (rx * ry)
                  - m
          ) / 2.0;
      } else {
          double h = 2.0 * m * n * Math.sqrt(d);
          double s = Math.signum(q + h)
                  * Math.pow(Math.abs(q + h), 1.0 / 3.0);
          double u = Math.signum(q - h)
                  * Math.pow(Math.abs(q - h), 1.0 / 3.0);

          double rx = -s - u - c * 4.0 + 2.0 * m2;
          double ry = (s - u) * Math.sqrt(3.0);
          double rm = Math.sqrt(rx * rx + ry * ry);

          co = (
                  ry / Math.sqrt(rm - rx)
                  + 2.0 * g / rm
                  - m
          ) / 2.0;
      }

      double sin = Math.sqrt(Math.max(0.0, 1.0 - co * co));

      double closestX = ax * co;
      double closestY = ay * sin;

      return Math.hypot(closestX - px, closestY - py)
              * Math.signum(py - closestY);
    }

    @Override
    public boolean contains(Translation2d point) {
        return sdEllipse(point) <= 0.0;
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
        double x = center.getX() + xAxis * Math.cos(angleRad);
        double y = center.getY() + yAxis * Math.sin(angleRad);
        Translation2d pointLocation = new Translation2d(x, y).rotateAround(center, rotation.unaryMinus());

        points[i] = new Pose2d(pointLocation, Rotation2d.kZero);
      }
      return points;
    }
  }
}