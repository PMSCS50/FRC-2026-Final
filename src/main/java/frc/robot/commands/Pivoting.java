package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Pivot;

import com.revrobotics.spark.ClosedLoopSlot;

public class Pivoting extends Command {

    private static final double SETPOINT_A = IntakeConstants.kPivotSetpointA; // 0.0 (up)
    private static final double SETPOINT_B = IntakeConstants.kPivotSetpointB; // down
    private static final double TOLERANCE = 0.05;

    private final Pivot pivot;
    private final double targetSetpoint;

    //private ClosedLoopSlot slot;

    public Pivoting(Pivot pivot, boolean forward) {
        this.pivot = pivot;
        this.targetSetpoint = forward ? SETPOINT_B : SETPOINT_A;
        //slot = forward ? ClosedLoopSlot.kSlot0 : ClosedLoopSlot.kSlot1;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.goToPosition(targetSetpoint);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("pivot location", pivot.getPivotPosition());
    }

    @Override
    public boolean isFinished() {
        return pivot.atPosition(targetSetpoint, TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
            if (interrupted) pivot.stopPivot(); // if not interrupted, position is already reached
    }
}