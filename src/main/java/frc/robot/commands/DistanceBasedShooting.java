package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;

public class DistanceBasedShooting extends Command {

    private final Shooter shooter;
    private final VisionSubsystem vision;
    private final int targetId;
    private final int leftId;
    private final int rightId;
    
    public DistanceBasedShooting(Shooter shooter, VisionSubsystem vision, int targetId, int leftId, int rightId) {
        this.shooter = shooter;
        this.vision = vision;
        this.targetId = targetId;
        this.leftId = leftId;
        this.rightId = rightId;
        addRequirements(shooter);
    }

    // Replace the single-tag constructor with:
    public DistanceBasedShooting(Shooter shooter, VisionSubsystem vision, int targetId) {
        this(shooter, vision, targetId, targetId, targetId);
    }

    // public DistanceBasedShooting(Shooter shooter, VisionSubsystem vision, int targetId) {
    //     this.shooter = shooter;
    //     this.vision = vision;
    //     this.targetId = targetId;

    //     addRequirements(shooter);
    // }


    @Override
    public void initialize() {}

    @Override
    public void execute() {
        SmartDashboard.putNumber("Distance to tag", vision.getDistance(targetId));
        SmartDashboard.putNumber("Shooter velocity", shooter.getVelocity());

        // if (vision.hasTarget(targetId) && !vision.hasTarget(leftId) && !vision.hasTarget(rightId)) {
        //     shooter.rpmControl(vision.getDistance(targetId));
        //     if (shooter.atCorrectRPM(targetId)) {
        //         shooter.spinKickers();
        //     }
        // } else if (vision.hasTarget(leftId) && vision.hasTarget(targetId)) {
        //     if (vision.getDistance(targetId) > vision.getDistance(leftId)) {
        //         shooter.rpmControl(vision.getDistance(leftId));
        //         if (shooter.atCorrectRPM(leftId)) {
        //             shooter.spinKickers();
        //         }

        //     } else {
        //         shooter.rpmControl(vision.getDistance(targetId));
        //         if (shooter.atCorrectRPM(targetId)) {
        //             shooter.spinKickers();
        //         }
        //     }

        // } else if (vision.hasTarget(rightId) && vision.hasTarget(targetId)) {
        //     if (vision.getDistance(targetId) > vision.getDistance(rightId)) {
        //         shooter.rpmControl(vision.getDistance(rightId));
        //         if (shooter.atCorrectRPM(rightId)) {
        //             shooter.spinKickers();
        //         }

        //     } else {
        //         shooter.rpmControl(vision.getDistance(targetId));
        //         if (shooter.atCorrectRPM(targetId)) {
        //             shooter.spinKickers();
        //         }
        //     }
        // } else if (vision.hasTarget(leftId) && !vision.hasTarget(targetId)) {
        //     shooter.rpmControl(vision.getDistance(leftId));
        //     if (shooter.atCorrectRPM(leftId)) {
        //         shooter.spinKickers();
        //     }
        // } else if (vision.hasTarget(rightId) && !vision.hasTarget(targetId)) {
        //     shooter.rpmControl(vision.getDistance(rightId));
        //     if (shooter.atCorrectRPM(rightId)) {
        //         shooter.spinKickers();
        //     }
        // }

        if (vision.hasTarget(targetId)) {
            shooter.rpmControl(vision.getDistance(targetId));
            if (shooter.atCorrectRPM(targetId)) {
                shooter.spinKickers();
            }
        } else if (vision.hasTarget(leftId) && !vision.hasTarget(targetId)) {
            shooter.rpmControl(vision.getDistance(leftId));
            if (shooter.atCorrectRPM(leftId)) {
                shooter.spinKickers();
            }
        } else if (vision.hasTarget(rightId) && !vision.hasTarget(targetId)) {
            shooter.rpmControl(vision.getDistance(rightId));
            if (shooter.atCorrectRPM(rightId)) {
                shooter.spinKickers();
            }
        }



        // if (vision.hasTarget(targetId)) {
        //     shooter.rpmControl(vision.getDistance(targetId));
        //     if (shooter.atCorrectRPM(targetId)) {
        //         shooter.spinKickers();
        //     }
        // } else {
        //     shooter.stop();
        // }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.stopKicker();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}