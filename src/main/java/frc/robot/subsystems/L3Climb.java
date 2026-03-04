package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.L3ClimbConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DigitalInput;

// THIS CLASS IS FOR OLIVER'S L3 CLIMB. REPLACE THE ORIGINAL CLIMB CLASS WITH THIS ONCE THE L3 CLIMB IS DONE

public class L3Climb extends SubsystemBase {
    //1 = left (facing forwards), 2 = right (facing forwards)

    private final SparkMaxConfig climbMotor1Config = new SparkMaxConfig();
    private final SparkMaxConfig climbMotor2Config = new SparkMaxConfig();
    private final SparkMaxConfig slideMotor1Config = new SparkMaxConfig();
    private final SparkMaxConfig slideMotor2Config = new SparkMaxConfig();

    private final SparkMax climbMotor1 = new SparkMax(L3ClimbConstants.climbMotor1CanId, MotorType.kBrushless);
    private final SparkMax climbMotor2 = new SparkMax(L3ClimbConstants.climbMotor2CanId, MotorType.kBrushless);

    private final SparkMax slideMotor1 = new SparkMax(L3ClimbConstants.slideMotor1CanId, MotorType.kBrushless);
    private final SparkMax slideMotor2 = new SparkMax(L3ClimbConstants.slideMotor2CanId, MotorType.kBrushless);

    private final RelativeEncoder climbEncoder = climbMotor1.getEncoder();
    private final RelativeEncoder sliderEncoder = slideMotor1.getEncoder(); 

    private final DigitalInput limitSwitchHook = new DigitalInput(1);
    private final DigitalInput limitSwitchTop = new DigitalInput(2);
    private final DigitalInput limitSwitchBottom = new DigitalInput(3);

    private String climbStatus = "climbDown";
    private String slideStatus = "in";
    private int climbLevel = 0;


    public L3Climb() {
        
        SparkMaxConfig[] configs = {climbMotor1Config, climbMotor2Config, slideMotor1Config, slideMotor2Config};
        
        for (SparkMaxConfig climbMotorConfig : configs) {
            climbMotorConfig
                //.inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);
        }
        climbMotor1.configure(climbMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climbMotor2.configure(climbMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        slideMotor1.configure(slideMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        slideMotor2.configure(slideMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        boolean inverted = true;

        //climbMotor2.follow(climbMotor1,inverted);
        //slideMotor2.follow(slideMotor1,inverted);

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climb limit switch HOOK", limitSwitchHook.get());
        SmartDashboard.putBoolean("Climb limit switch TOP", limitSwitchTop.get());
        SmartDashboard.putBoolean("Climb limit switch BOTTOM", limitSwitchBottom.get());
        SmartDashboard.putNumber(" Climb position", getDistance());
    }

    public void pullOuterArms(){
        if (limitSwitchHook.get()) {
            if (!limitSwitchTop.get()) {
                climbMotor1.set(0);
                if (climbStatus.equals("InnerArmsDoneHalfway")) {
                    climbLevel--;
                } else {
                    climbLevel++;
                }
                climbStatus = "OuterArmsDone";
            } else {       
                climbMotor1.set(L3ClimbConstants.climbSpeed);
            }
        }
    }
    
    public void pullInnerArms() {
        if (limitSwitchHook.get()) {
            if (!limitSwitchBottom.get()) {
                climbMotor1.set(0);
                climbLevel++;
                climbStatus = "InnerArmsDone";
            } else {
                climbMotor1.set(-L3ClimbConstants.climbSpeed);
            }
        }
    }

    //only for L1 during autonomous, so we can get down
    public void pullInnerArmsHalfway() {
        if (limitSwitchHook.get()) {
            if (getDistance() <= 15) {
                climbMotor1.set(0);
                climbStatus = "InnerArmsDoneHalfway";
            } else {
                climbMotor1.set(-L3ClimbConstants.climbSpeed);
            }
        }
    }

    public void slideOut() {
        if (getSliderDistance() >= 3.5) {
            slideMotor1.set(0);
            slideStatus = "out";
        } else {
            slideMotor1.set(L3ClimbConstants.slideSpeed);
        }
    }

    public void slideIn() {
        if (getSliderDistance() <= 0.5) {
            slideMotor1.set(0);
            slideStatus = "in";
        } else {
            slideMotor1.set(-L3ClimbConstants.slideSpeed);
        }
    }

    public void stopClimb() {
        climbMotor1.set(0);
    }

    public void reset(){
        climbMotor1.set(-L3ClimbConstants.climbSpeed);
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
    
    public void setClimbStatus(String status) {
        climbStatus = status;
    }

    public String getClimbStatus() {
        return climbStatus;
    }

    public int getClimbLevel() {
        return climbLevel;
    }

    public String getSlideStatus() {
        return slideStatus;
    }
    
    public double getDistance() {
        double climbMotorRadius = 0.125;
        double distance = climbEncoder.getPosition() * 2 * Math.PI * climbMotorRadius;
        return distance;
    }

    public double getSliderDistance() {
        double slideMotorRadius = 0.125;
        double distance = sliderEncoder.getPosition() * 2 * Math.PI * slideMotorRadius;
        return distance;
    }

    public double getVelocity() {
        return climbEncoder.getVelocity();
    }


}