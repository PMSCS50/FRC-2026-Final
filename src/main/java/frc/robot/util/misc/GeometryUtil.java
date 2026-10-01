// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.misc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.util.ArrayList;

/** Geometry utilities for working with translations, rotations, transforms, and poses. */
/** Also helps with polygon operations for Zones */
public class GeometryUtil {

  private GeometryUtil() {}

  /**
   * Creates a pure translating transform
   *
   * @param translation The translation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Translation2d translation) {
    return new Transform2d(translation, Rotation2d.kZero);
  }

  /**
   * Creates a pure translating transform
   *
   * @param x The x coordinate of the translation
   * @param y The y coordinate of the translation
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(double x, double y) {
    return new Transform2d(x, y, Rotation2d.kZero);
  }

  /**
   * Creates a pure rotating transform
   *
   * @param rotation The rotation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Rotation2d rotation) {
    return new Transform2d(Translation2d.kZero, rotation);
  }

  /**
   * Converts a Pose2d to a Transform2d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  public static Pose2d inverse(Pose2d pose) {
    Rotation2d rotationInverse = pose.getRotation().unaryMinus();
    return new Pose2d(
        pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
  }

  /**
   * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
   * chain
   *
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  public static Pose2d toPose2d(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Creates a pure translated pose
   *
   * @param translation The translation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d toPose2d(Translation2d translation) {
    return new Pose2d(translation, Rotation2d.kZero);
  }

  /**
   * Creates a pure rotated pose
   *
   * @param rotation The rotation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d toPose2d(Rotation2d rotation) {
    return new Pose2d(Translation2d.kZero, rotation);
  }

  /**
   * Multiplies a twist by a scaling factor
   *
   * @param twist The twist to multiply
   * @param factor The scaling factor for the twist components
   * @return The new twist
   */
  public static Twist2d multiply(Twist2d twist, double factor) {
    return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
  }

  /**
   * Converts a Pose3d to a Transform3d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform3d toTransform3d(Pose3d pose) {
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Converts a Transform3d to a Transform2d
   *
   * @param transform The original transform
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Transform3d transform) {
    return new Transform2d(
        transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d());
  }

  /**
   * Converts a Transform3d to a Pose3d to be used as a position or as the start of a kinematic
   * chain
   *
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  public static Pose3d toPose3d(Transform3d transform) {
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Converts a ChassisSpeeds to a Twist2d by extracting two dimensions (Y and Z). chain
   *
   * @param speeds The original translation
   * @return The resulting translation
   */
  public static Twist2d toTwist2d(ChassisSpeeds speeds) {
    return new Twist2d(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  /**
   * Creates a new pose from an existing one using a different translation value.
   *
   * @param pose The original pose
   * @param translation The new translation to use
   * @return The new pose with the new translation and original rotation
   */
  public static Pose2d withTranslation(Pose2d pose, Translation2d translation) {
    return new Pose2d(translation, pose.getRotation());
  }

  /**
   * Creates a new pose from an existing one using a different rotation value.
   *
   * @param pose The original pose
   * @param rotation The new rotation to use
   * @return The new pose with the original translation and new rotation
   */
  public static Pose2d withRotation(Pose2d pose, Rotation2d rotation) {
    return new Pose2d(pose.getTranslation(), rotation);
  }

  /**
   * Converts a WPILib Pose2d array into a java.awt.geom.Area object.
   * Skips null separators if present in the input.
   */
  private static Area createAreaFromPoses(Pose2d[] vertices) {
    if (vertices == null || vertices.length < 3) {
      return new Area();
    }
    Path2D.Double path = new Path2D.Double();
    boolean started = false;
    
    for (Pose2d vertex : vertices) {
      if (vertex == null) {
        if (started) {
          path.closePath();
          started = false;
        }
        continue;
      }
      
      if (!started) {
        path.moveTo(vertex.getX(), vertex.getY());
        started = true;
      } else {
        path.lineTo(vertex.getX(), vertex.getY());
      }
    }
    
    if (started) {
      path.closePath();
    }
    return new Area(path);
  }

  /**
   * Converts an Area back into a single flat Pose2d[] array.
   * Inserts a 'null' element between disjoint shapes or holes so AdvantageScope 
   * renders them as distinct boundaries without ugly cross-connecting lines.
   */
  private static Pose2d[] createFlatPosesFromArea(Area area) {
    ArrayList<Pose2d> flatList = new ArrayList<>();
    PathIterator iterator = area.getPathIterator(null);
    double[] coords = new double[6];

    while (!iterator.isDone()) {
      int type = iterator.currentSegment(coords);
      switch (type) {
        case PathIterator.SEG_MOVETO:
          if (!flatList.isEmpty()) {
            flatList.add(null);
          }
          flatList.add(new Pose2d(coords[0], coords[1], Rotation2d.kZero));
          break;
        case PathIterator.SEG_LINETO:
          flatList.add(new Pose2d(coords[0], coords[1], Rotation2d.kZero));
          break;
        case PathIterator.SEG_CLOSE:
          break;
      }
      iterator.next();
    }
    return flatList.toArray(new Pose2d[0]);
  }

  /**
   * Combines both shapes into a unified boundary array.
   */
  public static Pose2d[] union(Pose2d[] polyA, Pose2d[] polyB) {
    Area areaA = createAreaFromPoses(polyA);
    Area areaB = createAreaFromPoses(polyB);
    areaA.add(areaB);
    return createFlatPosesFromArea(areaA);
  }

  /**
   * Finds the overlapping area between both shapes as a single array.
   */
  public static Pose2d[] intersection(Pose2d[] polyA, Pose2d[] polyB) {
    Area areaA = createAreaFromPoses(polyA);
    Area areaB = createAreaFromPoses(polyB);
    areaA.intersect(areaB);
    return createFlatPosesFromArea(areaA);
  }

  /**
   * Subtracts PolyB from PolyA (A - B) as a single array.
   */
  public static Pose2d[] difference(Pose2d[] polyA, Pose2d[] polyB) {
    Area areaA = createAreaFromPoses(polyA);
    Area areaB = createAreaFromPoses(polyB);
    areaA.subtract(areaB);
    return createFlatPosesFromArea(areaA);
  }
}
