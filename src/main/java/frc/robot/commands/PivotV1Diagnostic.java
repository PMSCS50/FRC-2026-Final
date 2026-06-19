package frc.robot.commands;

import frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;

//First part of 2-part diagnostic test to find kS and kCos of our pivot
//MAKE SURE THAT THE PIVOT IS PERFECTLY HORIZONTAL BEFORE BEGINNING THIS TEST
public class PivotV1Diagnostic extends Command {

    private Pivot pivot;
    private LoggedNetworkNumber V1 = new LoggedNetworkNumber("Pivot/Diagnostics/V1");
    private double pivotSpeed = 0;
    private double lastPivotPosition = -1;

    public PivotV1Diagnostic(Pivot pivot) {
        this.pivot = pivot;
    }

    @Override
    public void initialize() {
        pivot.resetPivot();
    }

    @Override
    public void execute() {
        pivot.spinPivotDuty(pivotSpeed);
        if (pivot.getPivotPosition() <= lastPivotPosition) {
            V1.set(pivotSpeed);
        }
        lastPivotPosition = pivot.getPivotPosition();
        pivotSpeed -= 0.01;
    }

    @Override
    public boolean isFinished() {
        return V1.get() <= 0;
    }

}