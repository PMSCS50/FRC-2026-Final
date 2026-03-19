package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VisionSubsystem;


public class PivotingBack extends Command {

    private final Intake intake;

    private static final double SETPOINT_A = IntakeConstants.kPivotSetpointB;
    private static final double SETPOINT_B = IntakeConstants.kPivotSetpointA;
    private static final double TOLERANCE = 0.5;

    private double currentTarget;
    private int totalTrips;     
    private int tripsCompleted;

    public PivotingBack(Intake intake, int trips) {
        this.intake = intake;
        this.totalTrips = trips;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.resetPivot();
        tripsCompleted = 0;
        currentTarget = SETPOINT_B; 
        intake.goToPosition(currentTarget);
    }

    @Override
    public void execute() {
        if (intake.atPosition(currentTarget, TOLERANCE)) {
            tripsCompleted++;
            // flip target
            currentTarget = (currentTarget == SETPOINT_A) ? SETPOINT_B : SETPOINT_A;
            intake.goToPosition(currentTarget);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopPivot();
    }

    @Override
    public boolean isFinished() {
        return tripsCompleted >= totalTrips;
    }
}