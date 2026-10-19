package frc.robot.util.misc;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class VariableArmSim extends SingleJointedArmSim {
    private final DoubleSupplier m_lengthSupplier;
    private final double m_armMassKg; // You can adjust this based on your arm's mass
    private final DCMotor m_gearbox;
    private final double m_gearing;

    private final double m1;
    private final double m2;
    private final double m3;

    private final double l1;
    private final double l2;
    private final double l3;

    public VariableArmSim(double armMassKg, double[] masses, double[] lengths, DCMotor gearbox, double gearing, double startingMOI, DoubleSupplier lengthSupplier, double minAngle, double maxAngle) {
        // Pass a default fallback length to the super constructor
        super(LinearSystemId.createSingleJointedArmSystem(gearbox, armMassKg, gearing), 
            gearbox, gearing, lengthSupplier.getAsDouble(), minAngle, maxAngle, false, maxAngle, null); 
        
        this.m_lengthSupplier = lengthSupplier;
        this.m_gearbox = gearbox;
        this.m_armMassKg = armMassKg;
        this.m_gearing = gearing;

        this.m1 = masses[0];
        this.m2 = masses[1];
        this.m3 = masses[2];

        this.l1 = lengths[0];
        this.l2 = lengths[1];
        this.l3 = lengths[2];
    }

    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentX, Matrix<N1, N1> u, double dtSeconds) {
        double currentLength = m_lengthSupplier.getAsDouble();
        double currentAngleRad = currentX.get(0, 0); 

        double dynamicMOI = SingleJointedArmSim.estimateMOI(currentLength, m_armMassKg); 
        double gravityTorque = m_armMassKg * 9.80665 * (currentLength / 2.0) * Math.cos(currentAngleRad); 
        double accelerationFromGravity = gravityTorque / dynamicMOI;

        LinearSystem<N2, N1, N2> dynamicPlant = LinearSystemId.createSingleJointedArmSystem(m_gearbox, dynamicMOI, m_gearing); 
        Matrix<N2, N1> xDot = dynamicPlant.getA().times(currentX).plus(dynamicPlant.getB().times(u));
        xDot.set(1, 0, xDot.get(1, 0) + accelerationFromGravity);

        return currentX.plus(xDot.times(dtSeconds));
    }

    private double calculateMOI(double length) {
        return (1.0/3.0) * m1 * l1 * l1 + (1.0/12.0) * m2 * l2 * l2 + m2 * (length - l2 / 2.0) * (length - l2 / 2.0);
    }
}