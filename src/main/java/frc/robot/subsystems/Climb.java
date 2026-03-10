package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ClimbConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Climb extends SubsystemBase {
    //1 = left (facing forwards), 2 = right (facing forwards)

    private final DigitalInput limitSwitchHook = new DigitalInput(9);
    private final DigitalInput limitSwitchTop = new DigitalInput(8);
    private final DigitalInput limitSwitchBottom = new DigitalInput(7);

    private SparkMaxConfig climbMotorConfig = new SparkMaxConfig();
    private SparkMax climbMotor = new SparkMax(ClimbConstants.climbMotorCanId, MotorType.kBrushless);
    private RelativeEncoder climbEncoder = climbMotor.getEncoder(); 





    //private SparkMax CoralRoller2 = new SparkMax(CoralRollersConstants.coralRoller1CanId, MotorType.kBrushless);



    public Climb() {
        climbMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        
        

        
        
        

       climbMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }

    @Override
    public void periodic() {
        limitSwitchHook.get();
        SmartDashboard.putBoolean("Climb limit switch", limitSwitchHook.get());
        SmartDashboard.putNumber("Climb position", climbEncoder.getPosition());
    }

   public void pull(){
        if (limitSwitchHook.get()) {
            if (!limitSwitchTop.get()) {
                climbMotor.set(0);
            } else {       
                climbMotor.set(ClimbConstants.climbSpeed);
            }
        }
    }
    
    public void push() {
        if (limitSwitchBottom.get()) {
            climbMotor.set(0);
        } else {
            climbMotor.set(-ClimbConstants.climbSpeed);
        }
    }

    public void stopClimb() {
        climbMotor.set(0);
    }

    public void reset(){
        climbMotor.set(-ClimbConstants.climbSpeed);
    }
    
    public boolean getHookLimit() {
        return limitSwitchHook.get();
    }

    public boolean getBottomLimit() {
        return limitSwitchBottom.get();
    }

    public boolean getTopLimit() {
        return limitSwitchTop.get();
    }
    
    
    public double getDistance() {
        double climbMotorRadius = 0.689;
        double distance = climbEncoder.getPosition() * 2 * Math.PI * climbMotorRadius;
        return distance;
    }

    


    
}