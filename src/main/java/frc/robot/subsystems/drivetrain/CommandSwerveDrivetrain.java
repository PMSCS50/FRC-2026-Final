package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.VisionConstants;
import frc.robot.pathfinding.PPLogger;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 *  !Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 *  !Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem, DriveIO {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    //* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;

    //* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

    //* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private final DriveIOInputsAutoLogged m_inputs = new DriveIOInputsAutoLogged();

    /** // !Swerve request to apply during robot-centric path following 
     *  // *This also takes in our robot's physical constrants to create optimal path speeds.
    */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    //**Setpoint generator to optimize traversal through paths, made by FRC team 254.*/
    private SwerveSetpointGenerator m_setpointGenerator;

    //**A setpoint value */
    private SwerveSetpoint m_previousSetpoint;
    
    //** Swerve request to apply after the robot finished going through a path in Pathmaster. */
    private final SwerveRequest.Idle m_idle = new SwerveRequest.Idle();

    //* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    

    //* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)

            // *Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    //* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)

            // *Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * // !SysId routine for characterizing rotation.
     * // *This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * // *See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * // !Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * // *This constructs the underlying hardware devices, so users should not construct
     * // *the devices themselves. If they need the devices, they can access them through
     * // *getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        configureSignalLogging();
    }

    /**
     * // !Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * // *This constructs the underlying hardware devices, so users should not construct
     * // *the devices themselves. If they need the devices, they can access them through
     * // *getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * // !Constructs a CTRE SwerveDrivetrain using the specified constants.
     * 
     * *This constructs the underlying hardware devices, so users should not construct
     * *the devices themselves. If they need the devices, they can access them through
     * *getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    // *Configure the Autobuilder for auton paths and stuff
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            m_setpointGenerator = new SwerveSetpointGenerator(
                config,
                RotationsPerSecond.of(3).in(RadiansPerSecond) // max rotational velocity in rad/s
            );

            m_previousSetpoint = new SwerveSetpoint(
                getState().Speeds,
                getState().ModuleStates,
                DriveFeedforwards.zeros(config.numModules)
            );

            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds

                (speeds, feedforwards) -> {

                    // *Generate next setpoint. Thanks Cheesy Poofs.
                    m_previousSetpoint = m_setpointGenerator.generateSetpoint(
                        m_previousSetpoint,
                        speeds,
                        0.02
                    );

                    PPLogger.logVelocities(
                        Math.hypot(getState().Speeds.vxMetersPerSecond, getState().Speeds.vyMetersPerSecond),
                        Math.hypot(m_previousSetpoint.robotRelativeSpeeds().vxMetersPerSecond, m_previousSetpoint.robotRelativeSpeeds().vyMetersPerSecond),
                        getState().Speeds.omegaRadiansPerSecond,
                        m_previousSetpoint.robotRelativeSpeeds().omegaRadiansPerSecond
                    );

                    // *Apply generated module states
                    setControl(
                        m_pathApplyRobotSpeeds
                            .withSpeeds(m_previousSetpoint.robotRelativeSpeeds())
                            .withWheelForceFeedforwardsX(m_previousSetpoint.feedforwards().robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(m_previousSetpoint.feedforwards().robotRelativeForcesYNewtons())
                    );
                },
                new PPHolonomicDriveController(
                    // *PID constants for translation
                    new PIDConstants(8, 0, 0),
                    // *PID constants for rotation
                    new PIDConstants(4, 0, 0)
                ),
                config,
                // *Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError(
                "Failed to load PathPlanner config and configure AutoBuilder: " + ex.getMessage(),
                ex.getStackTrace()
            );
            throw new RuntimeException("RobotConfig load failed — deploy may be missing deploy/pathplanner/settings.json", ex);
        }
    }

    /**
     * *Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * *Runs the SysId Quasistatic test in the given direction for the routine
     * *specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * *Runs the SysId Dynamic test in the given direction for the routine
     * *specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    // *get robot pose
    public Pose2d getPose() {
        return getState().Pose;
    }

    public ChassisSpeeds getSpeeds() {
        return getState().Speeds;
    }

    public Command idle() {
        return applyRequest(() -> m_idle);
    }

    @Override
    public void periodic() {
        // *Update inputs; log inputs and other values in Advantagekit
        updateInputs(m_inputs);
        Logger.processInputs("LoggedDrivetrain", m_inputs);  // also needed for AdvantageKit to log it

        // |Swerve module states and motor outputs
        for (int i = 0; i < 4; i++) {
            SwerveModule<?, ?, ?> module = getModule(i);
            Logger.recordOutput("Drive/Module_" + (i+1) + "/Drivemotor/Voltage", module.getDriveMotor().getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Drive/Module_" + (i+1) + "/Drivemotor/SupplyCurrent", module.getDriveMotor().getSupplyCurrent().getValueAsDouble());
            Logger.recordOutput("Drive/Module_" + (i+1) + "/Drivemotor/StatorCurrent", module.getDriveMotor().getStatorCurrent().getValueAsDouble());

            Logger.recordOutput("Drive/Module_" + (i+1) + "/Turnmotor/Voltage", module.getSteerMotor().getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Drive/Module_" + (i+1) + "/Turnmotor/SupplyCurrent", module.getSteerMotor().getSupplyCurrent().getValueAsDouble());
            Logger.recordOutput("Drive/Module_" + (i+1) + "/Turnmotor/StatorCurrent", module.getSteerMotor().getStatorCurrent().getValueAsDouble());
        }

        // *Raw Pigeon2 Gyro Logging and Config
        Pigeon2 pigeon = getPigeon2();
        // |Orientation
        Logger.recordOutput("Gyro/RawYaw",   pigeon.getYaw().getValueAsDouble());
        Logger.recordOutput("Gyro/Pitch",    pigeon.getPitch().getValueAsDouble());
        Logger.recordOutput("Gyro/Roll",     pigeon.getRoll().getValueAsDouble());

        // |Angular rates — useful for catching MT2 rejections and tip-over events
        Logger.recordOutput("Gyro/YawRate",   pigeon.getAngularVelocityZWorld().getValueAsDouble());
        Logger.recordOutput("Gyro/PitchRate", pigeon.getAngularVelocityYWorld().getValueAsDouble());
        Logger.recordOutput("Gyro/RollRate",  pigeon.getAngularVelocityXWorld().getValueAsDouble());

        // |Health — tells you immediately if it browned out or rebooted mid-match
        Logger.recordOutput("Gyro/IsConnected",           pigeon.isConnected());
        Logger.recordOutput("Gyro/FaultHardware",         pigeon.getFault_Hardware().getValue());
        Logger.recordOutput("Gyro/FaultUndervoltage",     pigeon.getFault_Undervoltage().getValue());
        Logger.recordOutput("Gyro/FaultBootDuringEnable", pigeon.getFault_BootDuringEnable().getValue());

        if (Utils.isSimulation()) {
            if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
                DriverStation.getAlliance().ifPresent(allianceColor -> {
                    setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                            ? kRedAlliancePerspectiveRotation
                            : kBlueAlliancePerspectiveRotation
                    );
                    m_hasAppliedOperatorPerspective = true;
                });
            }
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        // *Run simulation at a faster rate so PID gains behave more reasonably
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            // *use the measured time delta, get battery voltage from WPILib
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private void configureSignalLogging() {
        for (int i = 0; i < getModules().length; i++) {
            var module = getModule(i);
            // *Phoenix automatically logs these at the CANivore rate
            // *Just ensure signal update frequency is set
            module.getDriveMotor().getStatorCurrent().setUpdateFrequency(50);
            module.getDriveMotor().getSupplyCurrent().setUpdateFrequency(50);
            module.getDriveMotor().getMotorVoltage().setUpdateFrequency(50);
            module.getSteerMotor().getStatorCurrent().setUpdateFrequency(50);
            module.getSteerMotor().getSupplyCurrent().setUpdateFrequency(50);
            module.getSteerMotor().getMotorVoltage().setUpdateFrequency(50);
        }
        SignalLogger.start(); // starts .hoot logging
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        var state = getState();

        inputs.moduleStates = state.ModuleStates;

        inputs.timestamp = Utils.getCurrentTimeSeconds();

        inputs.robotChassisSpeeds = state.Speeds;
        inputs.robotHeading = state.Pose.getRotation().getRadians();
        inputs.robotPose = state.Pose;
        
        // *For this example, we'll just set this to false. Implementing field-oriented control is left as an exercise to the user.
        inputs.isFieldOriented = false;
        inputs.distanceToHub = state.Pose.getTranslation().getDistance(VisionConstants.getHubPose().getTranslation());
    }
}
