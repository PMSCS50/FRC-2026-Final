package frc.robot.subsystems.swerve;

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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.Elastic;
import frc.robot.util.pathfinding.builders.GoingMerry;
import frc.robot.util.simulation.MapleSimSwerveDrivetrain;

/**
 *  !Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 *  !Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem, SwerveIO {
    private static final double kSimLoopPeriod = 0.002; // 2 ms
    private Notifier m_simNotifier = null;

    private int loggingLoopCounter = 0;
    private static final int LOG_EVERY_N_LOOPS = 5; // 5 loops = about 100ms
    //private final BaseStatusSignal[] m_logSignals;

    //* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;

    //* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

    //* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveIOInputsAutoLogged m_inputs = new SwerveIOInputsAutoLogged();

    /** // !Swerve request to apply during robot-centric path following 
     *  // *This also takes in our robot's physical constrants to create optimal path speeds.
    */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    // *Separate request object for runVelocity (teleop/default command).
    // *MUST NOT be the same instance as m_pathApplyRobotSpeeds: the path follower
    // *mutates m_pathApplyRobotSpeeds with .withWheelForceFeedforwardsX/Y at every
    // *step. Those feedforward forces persist on the shared object even after the path
    // *ends. runVelocity() only calls .withSpeeds() and never clears the feedforwards,
    // *so the stale path-end forces keep the drive motors spinning at cruise speed
    // *regardless of what velocity is commanded — causing the observed drift.
    private final SwerveRequest.ApplyRobotSpeeds m_teleopApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    
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
        super(drivetrainConstants, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));

        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
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
        //m_logSignals = cacheLogSignals();
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
        //m_logSignals = cacheLogSignals();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    // *Configure the Autobuilder for auton paths and stuff
    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            //Configure AutoBuilder
            AutoBuilder.configure(
                this::getPose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                this::getSpeeds, // Supplier of current robot speeds

                (speeds, feedforwards) -> {
                    setControl(
                        m_pathApplyRobotSpeeds.withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                    );
                },

                new PPHolonomicDriveController(
                    // *PID constants for translation
                    //Ligthened from 8.0 for sharp turns
                    new PIDConstants(7.5, 0, 0),
                    // *PID constants for rotation
                    new PIDConstants(4, 0, 0)
                ),
                config,
                // *Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );

            //GoingMerry is an AutoBuilder completely optimized for the type of pathfinding we are doing.
            //Unfortunately I cant copy the AutoBuilder configs so I'm stuck with this.
            //Also, since AutoBuilder is still used in the autochooser for auton paths I cant and wont remove it
            //The constructor is entirely the same except for the lack of a resetPose() parameter
            GoingMerry.configure(
                this::getPose,   // Supplier of current robot pose
                this::getSpeeds, // Supplier of current robot speeds

                (speeds, feedforwards) -> {
                    setControl(
                        m_pathApplyRobotSpeeds.withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
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
            Elastic.sendNotification(
                new Elastic.Notification()
                .withLevel(Elastic.NotificationLevel.ERROR)
                .withTitle("AutoBuilder Config Error") 
                .withDescription("AutoBuilder failed to configure properly. Most likely a RobotConfig error"));
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
        if (Utils.isSimulation() && mapleSimSwerveDrivetrain != null) {
            return mapleSimSwerveDrivetrain.mapleSimDrive
                .getSimulatedDriveTrainPose();
        }

        return getState().Pose;
    }

    public ChassisSpeeds getSpeeds() {
        if (Utils.isSimulation() && mapleSimSwerveDrivetrain != null) {
            return mapleSimSwerveDrivetrain.mapleSimDrive
                .getDriveTrainSimulatedChassisSpeedsRobotRelative();
        }

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
        Logger.recordOutput("maplesimpose", mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());

        boolean shouldLogSlowSignals = (++loggingLoopCounter % LOG_EVERY_N_LOOPS) == 0;

        // |Swerve module states and motor outputs
        if (shouldLogSlowSignals) {
            //BaseStatusSignal.refreshAll(m_logSignals);
            for (int i = 0; i < 4; i++) {
                SwerveModule<?, ?, ?> module = getModule(i);

                Logger.recordOutput("Drive/Module_" + (i+1) + "/Drivemotor/Voltage", module.getDriveMotor().getMotorVoltage().getValueAsDouble());
                Logger.recordOutput("Drive/Module_" + (i+1) + "/Drivemotor/SupplyCurrent", module.getDriveMotor().getSupplyCurrent().getValueAsDouble());
                Logger.recordOutput("Drive/Module_" + (i+1) + "/Drivemotor/StatorCurrent", module.getDriveMotor().getStatorCurrent().getValueAsDouble());

                Logger.recordOutput("Drive/Module_" + (i+1) + "/Turnmotor/Voltage", module.getSteerMotor().getMotorVoltage().getValueAsDouble());
                Logger.recordOutput("Drive/Module_" + (i+1) + "/Turnmotor/SupplyCurrent", module.getSteerMotor().getSupplyCurrent().getValueAsDouble());
                Logger.recordOutput("Drive/Module_" + (i+1) + "/Turnmotor/StatorCurrent", module.getSteerMotor().getStatorCurrent().getValueAsDouble());
            }

            // |Raw Pigeon2 gyro logging
            Pigeon2 pigeon = getPigeon2();
            //Yaw from -180 to 180 degrees
            Logger.recordOutput("Gyro/ModYaw", pigeon.getYaw().getValueAsDouble() % 360 - 180);
            Logger.recordOutput("Gyro/YawRate", pigeon.getAngularVelocityZWorld().getValueAsDouble());
            Logger.recordOutput("Gyro/IsConnected", pigeon.isConnected());
        }

        // |Sim alliance perspective config
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

    private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    
    @SuppressWarnings("unchecked")
    private void startSimThread() {
        mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
                this,
                Seconds.of(kSimLoopPeriod),
                Pounds.of(115),
                Meters.of(0.858),
                Meters.of(0.858),
                DCMotor.getKrakenX60(1),
                DCMotor.getKrakenX60(1),
                1.7,
                getModuleLocations(),
                getPigeon2(),
                getModules(),
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);

        resetPose(new Pose2d(
            2.0,
            2.0,
            Rotation2d.fromDegrees(0)
        ));

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void updateInputs(SwerveIOInputs inputs) {
        var state = getState();

        inputs.moduleStates = state.ModuleStates;

        inputs.timestamp = Utils.getCurrentTimeSeconds();

        inputs.robotChassisSpeeds = state.Speeds;
        inputs.robotHeading = state.Pose.getRotation().getRadians();

        inputs.totalCurrent = 0.0;

        for (int i = 0; i < 4; i++) {
            var module = getModule(i);
            inputs.totalCurrent += module.getDriveMotor().getSupplyCurrent().getValueAsDouble() + module.getSteerMotor().getSupplyCurrent().getValueAsDouble();
        }

        inputs.totalVoltage = RobotController.getBatteryVoltage();
        
        inputs.isFieldOriented = false;

        inputs.robotPose = state.Pose;
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }


    public void runVelocity(ChassisSpeeds speeds) {
        // discretize corrects for the robot rotating during the 20ms loop:
        // without it, translating + rotating simultaneously causes lateral drift
        // because the robot-relative velocity direction becomes stale mid-loop.
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

        // Use m_teleopApplyRobotSpeeds, NOT m_pathApplyRobotSpeeds.
        // The path follower leaves stale wheel-force feedforwards on m_pathApplyRobotSpeeds
        // after every path step. Those forces persist and override the velocity PID,
        // causing the robot to keep driving at path cruise speed even when commanded zero.
        setControl(
            m_teleopApplyRobotSpeeds.withSpeeds(discreteSpeeds)
        );
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) {mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);}
        super.resetPose(pose);
    }
}
