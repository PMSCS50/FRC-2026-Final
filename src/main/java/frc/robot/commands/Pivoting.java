package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class Pivoting extends Command {

    private static final double SETPOINT_A = IntakeConstants.kPivotSetpointA; // 0.0 (up)
    private static final double SETPOINT_B = IntakeConstants.kPivotSetpointB; // down
    private static final double TOLERANCE = 0.05;

    

    private final Intake intake;
    private final double targetSetpoint;

    public Pivoting(Intake intake, boolean forward) {
        this.intake = intake;
        this.targetSetpoint = forward ? SETPOINT_B : SETPOINT_A;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.goToPosition(targetSetpoint);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("pivot location", intake.getPivotEncoder().getPosition());
    }

    @Override
    public boolean isFinished() {
        return intake.atPosition(targetSetpoint, TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        intake.goToPosition(intake.getPivotPosition()); // hold position
    }
}