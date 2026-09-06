package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;

public class PivotToAngle extends Command {

    private final Pivot pivot;
    private final double targetRad;
    private final double toleranceRad;
    private final double setpointBRotations = 17;

    public PivotToAngle(Pivot pivot, boolean forward) {
        this.pivot = pivot;

        // Convert old rotation setpoints to radians
        this.targetRad = forward ? setpointBRotations * 2*Math.PI : 0.0;
        this.toleranceRad = 0.05 * 2*Math.PI; // old tolerance converted
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.setPivotAngle(targetRad);
    }

    @Override
    public boolean isFinished() {
        return pivot.atAngle(targetRad, toleranceRad);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) pivot.stop();
    }
}

