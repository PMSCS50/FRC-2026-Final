package frc.robot.subsystems.telescoping_arm.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.robot.Constants.ArmConstants;;

public class WristIOSim implements WristIO {

    private double appliedVolts = 0.0;
    private PIDController wristController;
    private boolean motionMagic = false;
    private double setpoint = 0.0;

    private final DCMotor krakenx44 = new DCMotor(
        12,
        4.05,
        275,
        1.4,
        Units.rotationsPerMinuteToRadiansPerSecond(7530),
        1
    ); // Kraken X44;
    private final SingleJointedArmSim wristSim;
    
    public WristIOSim() {
        wristController = new PIDController(
            ArmConstants.wristkP,
            ArmConstants.wristkI,
            ArmConstants.wristkD
        );

        wristSim = new SingleJointedArmSim(
            krakenx44,
            ArmConstants.wristGearing, // gear ratio
            0.0025163795, // moment of inertia
            .5, // arm length
            Math.toRadians(60), // smallest angle
            Math.toRadians(260), // largest angle
            true,   // simulate gravity
            Math.PI/2     // starting angle
        );
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        wristSim.update(0.02);

        inputs.position = wristSim.getAngleRads();
        inputs.velocity = wristSim.getVelocityRadPerSec();
        inputs.acceleration = 0.0; //sim does not expose accel
        inputs.appliedVoltage = appliedVolts;

        if (motionMagic) {
            double volts = MathUtil.clamp(wristController.calculate(inputs.position, setpoint), -12.0, 12.0);
            wristSim.setInputVoltage(volts);
        }
    }

    private void setVoltage(double volts) {
        motionMagic = false;
        appliedVolts = volts;
        wristSim.setInputVoltage(volts);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        setVoltage(dutyCycle * 12.0);
    }

    @Override
    public void setMotionMagicPosition(double positionDegrees) {
        if (!motionMagic) {
            wristController.reset();
            motionMagic = true; //lazy adjustment i think
        }
        setpoint = Math.toRadians(positionDegrees);
    }

    @Override
    public void stop() {
        setVoltage(0);
    }
    
}
