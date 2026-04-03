package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class Pivoting extends Command {

    private static final double SETPOINT_A = IntakeConstants.kPivotSetpointA; // 0.0 (up)
    private static final double SETPOINT_B = IntakeConstants.kPivotSetpointB; // down
    private static final double TOLERANCE = 0.05;

    

    private final Pivot pivot;
    private final double targetSetpoint;

    public Pivoting(Pivot pivot, boolean forward) {
        this.pivot = pivot;
        this.targetSetpoint = forward ? SETPOINT_B : SETPOINT_A;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {

        pivot.goToPosition(targetSetpoint);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("pivot location", pivot.getPivotEncoder().getPosition());
    }

    @Override
    public boolean isFinished() {
        return pivot.atPosition(targetSetpoint, TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        pivot.goToPosition(targetSetpoint); 
    }
}