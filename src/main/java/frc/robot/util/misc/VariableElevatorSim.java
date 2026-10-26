package frc.robot.util.misc;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ArmConstants;

public class VariableElevatorSim extends ElevatorSim {
    private final boolean simulateGravity;
    private final double minHeight;
    private final double maxHeight;
    private DoubleSupplier angleSupplier;

    public VariableElevatorSim(
        DCMotor gearbox,
        double gearing,
        double carriageMass,
        double drumRadius,
        double minHeight,
        double maxHeight,
        boolean simulateGravity,
        double startingHeight,
        DoubleSupplier angleSupplier,
        double... measurementStdDevs) {
        super(gearbox, gearing, carriageMass, drumRadius, minHeight, maxHeight, false, startingHeight, measurementStdDevs);
        this.simulateGravity = simulateGravity;
        this.minHeight = minHeight;
        this.maxHeight = maxHeight;
        this.angleSupplier = angleSupplier;
    }

    public VariableElevatorSim(
        double kV,
        double kA,
        DCMotor gearbox,
        double minHeight,
        double maxHeight,
        boolean simulateGravity,
        double startingHeight,
        DoubleSupplier angleSupplier,
        double... measurementStdDevs) {
        super(kV, kA, gearbox, minHeight, maxHeight, false, startingHeight, measurementStdDevs);
        this.simulateGravity = simulateGravity;
        this.minHeight = minHeight;
        this.maxHeight = maxHeight;
        this.angleSupplier = angleSupplier;
    }

    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dt) {
        // Calculate updated x-hat from Runge-Kutta.
        var updatedXhat =
            NumericalIntegration.rkdp(
                (x, _u) -> {
                Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
                if (simulateGravity) {
                    xdot = xdot.plus(VecBuilder.fill(0, -9.8 * Math.sin(angleSupplier.getAsDouble())));
                }
                return xdot;
                },
                currentXhat,
                u,
                dt);
        
        ArmConstants.elevatorExtension.set(
            MathUtil.clamp(updatedXhat.get(0, 0), minHeight, maxHeight)
        );

        // We check for collisions after updating x-hat.
        if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(minHeight, 0);
        }
        if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(maxHeight, 0);
        }

        return updatedXhat;
    } 
}
