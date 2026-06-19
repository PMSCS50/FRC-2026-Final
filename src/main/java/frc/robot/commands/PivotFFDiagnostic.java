package frc.robot.commands;

import frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;

//Second part of 2-part diagnostic test to find kS and kCos of our pivot
//Actually the part that gets us kS and kCos to put on the robot
//MAKE SURE THAT THE PIVOT IS PERFECTLY HORIZONTAL BEFORE BEGINNING THIS TEST
public class PivotFFDiagnostic extends Command {

    private Pivot pivot;
    private LoggedNetworkNumber V1 = new LoggedNetworkNumber("Pivot/Diagnostics/V1"); 
    private LoggedNetworkNumber V2 = new LoggedNetworkNumber("Pivot/Diagnostics/V2");
    private LoggedNetworkNumber kS = new LoggedNetworkNumber("Pivot/Diagnostics/kS");
    private LoggedNetworkNumber kCos = new LoggedNetworkNumber("Pivot/Diagnostics/kCos");

    private double pivotSpeed = V1.get() - 0.01;
    private double lastPivotPosition = -1;

    public PivotFFDiagnostic(Pivot pivot) {
        this.pivot = pivot;
    }

    @Override
    public void initialize() {
        pivot.resetPivot();
        V1.set(0.0); //REPLACE 0.0 WITH V1 FOUND IN PART ONE OF THE DIAGNOSTICS TEST
    }

    @Override
    public void execute() {
        pivot.spinPivotDuty(pivotSpeed);
        if (pivot.getPivotPosition() <= lastPivotPosition) {
            V2.set(pivotSpeed);
        }
        lastPivotPosition = pivot.getPivotPosition();
        pivotSpeed += 0.01;
    }

    @Override
    public boolean isFinished() {
        return V2.get() != 0;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            double ks = (V1.get() - V2.get()) / 2;
            kS.set(ks);

            double kcos = V2.get() + ks;
            kCos.set(kcos);
        }
    }

}