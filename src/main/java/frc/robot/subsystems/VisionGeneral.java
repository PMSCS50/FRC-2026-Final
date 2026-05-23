package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class VisionGeneral extends SubsystemBase {
    /**
     * @return Whether the vision system has a valid target
     */
    public abstract boolean hasTargets();

    /**
     * @param i 
     * @return The distance
     */
    public abstract int getDistance(int i);

    /**
     * @param i 
     * @return The x value
     */
    public abstract int getX(int i);
    /**
     * @param i 
     * @return The y value
     */
    public abstract int getY(int i);
    /**
    * @param i 
    * @return The yaw angle of the target, in radians
    */
    public abstract int getYawRad(int i);

    public abstract double getDistanceToTarget(Pose2d hubPose);
}
