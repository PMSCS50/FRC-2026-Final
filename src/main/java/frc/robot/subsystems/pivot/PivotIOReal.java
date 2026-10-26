package frc.robot.subsystems.pivot;


import static edu.wpi.first.units.Units.Amps;


import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import frc.robot.Constants.IntakeConstants;
import frc.robot.util.misc.VirtualPD;
import frc.robot.util.tunable.TunableControls.*;


public class PivotIOReal implements PivotIO {
   private final SparkMax motor = new SparkMax(IntakeConstants.pivotMotorCanId, MotorType.kBrushless);
   private final RelativeEncoder encoder = motor.getEncoder();
   private final SparkMaxConfig motorConfig = new SparkMaxConfig();


   private double setpoint = 0.0;
   private boolean pidControlled = false;


   TunablePIDController pidController = new TunablePIDController(
       new TunableControlConstants(
           "PivotControls",
           new ControlConstants()
           .withPID(0.1, 0, 0)
           .withTolerance(1.0))
   );

   public PivotIOReal() {
       motorConfig.inverted(true)
             .idleMode(IdleMode.kBrake)
             .smartCurrentLimit(40)
             .closedLoopRampRate(1);


       motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }


   @Override
   public void registerMotors() {
       VirtualPD.registerMotor(() -> Amps.of(motor.getOutputCurrent()), "Pivot");
   }


   @Override
   public void setVoltage(double volts) {
       pidControlled = false;
       motor.setVoltage(volts);
   }


   @Override
   public void setDutyCycle(double duty) {
       pidControlled = false;
       motor.set(duty);
   }


   @Override
   public void setPosition(double rotations) {
       pidControlled = true;
       setpoint = rotations;
   }


   @Override
   public void setEncoderPosition(double pos) {
       encoder.setPosition(pos);
   }


   @Override
   public void updateInputs(PivotIOInputs inputs) {
       pidController.updateParams();




       inputs.motorVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
       inputs.motorPosition = encoder.getPosition();
       inputs.motorVelocity = encoder.getVelocity();


       if (pidControlled) {
           motor.setVoltage(
               pidController.calculate(setpoint - inputs.motorPosition)
           );
       }
   }
}



