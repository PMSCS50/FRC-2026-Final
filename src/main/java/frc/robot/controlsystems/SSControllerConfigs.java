package frc.robot.controlsystems;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;

//I dont think records are in this version of wpilib. So, we need to make a class instead

/*
 * Class that stores the configuration values for SSController, 
 * which includes everything EXCEPT the system.
 * You can pass this class into SSController directly.
 * This way we can put an SSControllerConfigs inside the constants for each subsystem.
 * Then it just makes initialization easier. 
 * 
 * Normally this can be automated with a record, but since records seem to not be in
 * this version of java, we have to make it by hand. Stupid boilerplate code.
 */

public class SSControllerConfigs<States extends Num, Inputs extends Num, Outputs extends Num> {
    
    private final Nat<States> statesNat;
    private final Nat<Outputs> outputsNat;
    private final Vector<States> qCost;
    private final Vector<Inputs> rCost;
    private final Matrix<States, N1> modelStdDevs;
    private final Matrix<Outputs, N1> encoderStdDevs;
    private final double maxVoltage;
    private final Matrix<States, N1> tolerance;
    private final double loopPeriodSecs;

    public SSControllerConfigs(
            Nat<States> statesNat,
            Nat<Outputs> outputsNat,
            Vector<States> qCost,
            Vector<Inputs> rCost,
            Matrix<States, N1> modelStdDevs,
            Matrix<Outputs, N1> encoderStdDevs,
            double maxVoltage,
            Matrix<States, N1> tolerance,
            double loopPeriodSecs) {

        this.statesNat = statesNat;
        this.outputsNat = outputsNat;
        this.qCost = qCost;
        this.rCost = rCost;
        this.modelStdDevs = modelStdDevs;
        this.encoderStdDevs = encoderStdDevs;
        this.maxVoltage = maxVoltage;
        this.tolerance = tolerance;
        this.loopPeriodSecs = loopPeriodSecs
    }

    public Nat<States> getStatesNat() {
        return statesNat;
    }

    public Nat<Outputs> getOutputsNat() {
        return outputsNat;
    }

    public Vector<States> getQCost() {
        return qCost;
    }

    public Vector<Inputs> getRCost() {
        return rCost;
    }

    public double getMaxVoltage() {
        return maxVoltage;
    }

    public Matrix<States, N1> getTolerance() {
        return tolerance;
    }

    public double getLoopPeriodSecs() {
        return loopPeriodSecs;
    }
}