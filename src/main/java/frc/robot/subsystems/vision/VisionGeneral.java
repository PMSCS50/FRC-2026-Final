package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class VisionGeneral extends SubsystemBase {
    /**
     * @return Whether the vision system has a valid target
     */
    public abstract boolean hasTargets();

    /**
     * 
     * @return The distance
     */
    public abstract double getDistance();

    /**
     * @param i 
     * @return The x value
     */
    public abstract double getX();
    /**
     * @param i 
     * @return The y value
     */
    public abstract double getY();
    /**
    * @param i 
    * @return The yaw angle of the target, in radians
    */
    public abstract double getYawRad();

    public abstract double getDistanceToTarget(Pose2d hubPose);
}
