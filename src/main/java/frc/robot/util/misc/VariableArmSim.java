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

    private final double[] masses;
    private final double[] lengths;

    private final boolean gravity;

    private double debugVal;

    public VariableArmSim(double[] masses, double[] lengths, DCMotor gearbox, double gearing, double startingMOI, DoubleSupplier lengthSupplier, double minAngle, double maxAngle, boolean gravity) {
        // Pass a default fallback length to the super constructor
        super(LinearSystemId.createSingleJointedArmSystem(gearbox, startingMOI, gearing), 
            gearbox, gearing, lengthSupplier.getAsDouble(), minAngle, maxAngle, false, maxAngle, new double[] {0, 0}); 
        
        this.m_lengthSupplier = lengthSupplier;
        this.m_gearbox = gearbox;
        this.m_armMassKg = masses[0] + masses[1] + masses[2]; // Total mass of the arm
        this.m_gearing = gearing;

        this.masses = masses;
        this.lengths = lengths;

        this.gravity = gravity;
    }

    public double getLengthMeters() {
        return m_lengthSupplier.getAsDouble();
    }

    public double getDebugVal() {
        return debugVal;
    }

    public double calculateFF(double setpointRads) {
        double torque = m_armMassKg * 9.80665 * calculateCOM(m_lengthSupplier.getAsDouble(), masses, lengths) * Math.cos(getAngleRads());
        return torque * m_gearbox.rOhms / (m_gearbox.KtNMPerAmp * m_gearing);
    }

    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentX, Matrix<N1, N1> u, double dtSeconds) {
        double currentLength = m_lengthSupplier.getAsDouble();
        double currentAngleRad = currentX.get(0, 0); 

        double dynamicMOI = calculateMOI(m_lengthSupplier.getAsDouble(), masses, lengths);
        double gravityTorque = m_armMassKg * 9.80665 * calculateCOM(currentLength, masses, lengths) * Math.cos(currentAngleRad); 
        double accelerationFromGravity = gravity ? gravityTorque/dynamicMOI : 0.0;
        debugVal = gravityTorque/dynamicMOI;

        LinearSystem<N2, N1, N2> dynamicPlant = LinearSystemId.createSingleJointedArmSystem(m_gearbox, dynamicMOI, m_gearing); 
        Matrix<N2, N1> xDot = dynamicPlant.getA().times(currentX).plus(dynamicPlant.getB().times(u));
        xDot.set(1, 0, xDot.get(1, 0) - accelerationFromGravity);

        return currentX.plus(xDot.times(dtSeconds));
    }

    public static double calculateCOM(double length, double[] masses, double[] lengths) {
        double m1 = masses[0];
        double m2 = masses[1];
        double m3 = masses[2];

        double l1 = lengths[0];
        double l2 = lengths[1];
        double l3 = lengths[2];

        double x = (length - l3) / 2.0;

        double x1 = l1 / 2.0;
        double x2 = x + l2 / 2.0;
        double x3 = 2.0 * x + l3 / 2.0;

        return (m1*x1 + m2*x2 + m3*x3)
                / (m1 + m2 + m3);
    }

    public static double calculateMOI(double length, double[] masses, double[] lengths) {
        double m1 = masses[0];
        double m2 = masses[1];
        double m3 = masses[2];

        double l1 = lengths[0];
        double l2 = lengths[1];
        double l3 = lengths[2];

        return (1.0/3.0)*m1*l1*l1 + (1.0/3.0)*m2*l2*l2 + m2*((length - l3)/2)*((length-l3)/2) + (1.0/3.0)*m3*l3*l3 + m3*(length-l3)*(length-l3);
    }
}
