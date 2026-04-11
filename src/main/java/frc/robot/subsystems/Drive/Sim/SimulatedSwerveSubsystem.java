package frc.robot.subsystems.Drive.Sim;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.orchestration.HubOrchestrator;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.util.Alert;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Constants.Status;
import frc.util.Constants.Swerve;
import frc.util.Helpers;
import frc.util.ZoneConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily be used in
 * command-based projects.
 */
public class SimulatedSwerveSubsystem extends SimulatedSwerveBase implements SwerveSubsystem {
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.kZero;
    private boolean m_hasAppliedOperatorPerspective = false;

    private Field2d m_currentField;
    private Field2d m_targetField;

    private HolonomicDriveController m_driveController;

    private Field2d m_targetCenterPoseField;

    private Pose2d m_arcLockCenter;
    private double m_arcLockDistance;
    private double m_arcLockTheta;
    
    private boolean m_hubLock = false;
    
    private boolean m_trenchLock = false;
    
    private double m_trenchYPos = 0;

    private boolean m_isMoving = false;

    /** 0% - 100% of max speed */
    public double m_speedMultipler = 1.0; 

    private boolean m_ensureTheta = false;
    private DoubleSupplier m_ensuredThetaSupplier = () -> 0;

    DoubleSupplier m_optimalRotationSupplier = () -> 0;

    private boolean m_fieldCentric;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices themselves. If they
     * need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public SimulatedSwerveSubsystem(
            SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    public SimulatedSwerveSubsystem() {
        super(Constants.Swerve.kDrivetrainConstants);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices themselves. If they
     * need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to 0 Hz, this is 250
     *     Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param modules Constants for each specific module
     */
    public SimulatedSwerveSubsystem(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices themselves. If they
     * need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to 0 Hz, this is 250
     *     Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation in the form [x, y, theta]ᵀ, with
     *     units in meters and radians
     * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y, theta]ᵀ, with
     *     units in meters and radians
     * @param modules Constants for each specific module
     */
    public SimulatedSwerveSubsystem(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                odometryStandardDeviation,
                visionStandardDeviation,
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> currentPose(), // Supplier of current robot pose
                    this::resetControl, // Consumer for seeding pose against auto
                    () -> getSpeed(), // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> m_mapleSimSwerveDrivetrain.mapleSimDrive.setRobotSpeeds(speeds),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(Constants.Swerve.XYPID.kP, Constants.Swerve.XYPID.kI, Constants.Swerve.XYPID.kD),
                            // PID constants for rotation
                            new PIDConstants(Constants.Swerve.ThetaPID.kP, Constants.Swerve.XYPID.kI, Constants.Swerve.XYPID.kD)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
                    );
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> {
            this.setControl(requestSupplier.get());
        });
    }

    public Command driveWithJoysticks(DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
        return new CommandBuilder(this)
        .onExecute(() -> {
            double actualSpeedMultiplier = m_speedMultipler;
            if (Helpers.isBypassModeEnabled()) actualSpeedMultiplier = 1d;

            double vx = -leftY.getAsDouble() * Constants.Swerve.kMaxSpeed * actualSpeedMultiplier;
            double vy = -leftX.getAsDouble() * Constants.Swerve.kMaxSpeed * actualSpeedMultiplier;
            double vRot = -rightX.getAsDouble() * Constants.Swerve.kMaxAngularRate * actualSpeedMultiplier;

            if (m_hubLock) {
                m_arcLockCenter = Helpers.allianceHub();

                Rotation2d targetAngle = new Rotation2d(m_optimalRotationSupplier.getAsDouble() - Math.PI/2 - getShooterAngleCompensation());

                vRot = m_driveController.calculate(
                        currentPose(),
                        new Pose2d(currentPose().getTranslation(), targetAngle),
                        0,
                        targetAngle
                    ).omegaRadiansPerSecond * Swerve.kMaxAngularRate;
            }

            if (m_trenchLock) {
                Translation2d currentTranslation = currentPose().getTranslation();

                Rotation2d rotation = currentPose().getRotation();

                Pose2d target = new Pose2d(new Translation2d(currentTranslation.getX(), m_trenchYPos), rotation);

                vy = m_driveController.calculate(
                        currentPose(),
                        target,
                        0,
                        rotation
                    ).vyMetersPerSecond * Swerve.kMaxSpeed;

                if (m_fieldCentric && Math.abs(rotation.getDegrees()) < 90) {
                    vy *= -1;
                }

                m_targetField.setRobotPose(target);
            }

            if (m_ensureTheta) {
                vRot = m_driveController.getThetaController().calculate(currentPose().getRotation().getDegrees(), m_ensuredThetaSupplier.getAsDouble());
            }

            if (m_fieldCentric) {
                m_mapleSimSwerveDrivetrain.mapleSimDrive.setRobotSpeeds(
                    new ChassisSpeeds(
                        vy * (Helpers.isBlueAlliance() ? 1 : -1),
                        vx * (Helpers.isBlueAlliance() ? 1 : -1),
                        vRot
                    )
                );
            } else {
                m_mapleSimSwerveDrivetrain.mapleSimDrive.setRobotSpeeds(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        vy,
                        -vx,
                        vRot,
                        m_mapleSimSwerveDrivetrain.mapleSimDrive.getGyroSimulation().getGyroReading()
                    )
                );
            }
        });
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        m_currentField.setRobotPose(currentPose());

        SmartDashboard.putNumber("BatteryVoltage", RobotController.getBatteryVoltage());
        SmartDashboard.putData("Drive/OdometryPose", m_currentField);
        SmartDashboard.putNumber("Drive/vx", getSpeed().vxMetersPerSecond);
        SmartDashboard.putNumber("Drive/vy", getSpeed().vyMetersPerSecond);
        SmartDashboard.putNumber("Drive/vRot", getSpeed().omegaRadiansPerSecond);
        SmartDashboard.putNumber("SOTM / Drive Interpolated Theta", m_optimalRotationSupplier.getAsDouble());
        SmartDashboard.putNumber("SOTM / Drive Current Theta", currentPose().getRotation().getRadians());
    }

    private MapleSimSwerveDrivetrain m_mapleSimSwerveDrivetrain = null;

    private void startSimThread() {
        m_currentField = new Field2d();

        m_driveController = new HolonomicDriveController(
            Swerve.kHolonomicXPIDController,
            Swerve.kHolonomicYPIDController,
            Swerve.kHolonomicThetaPIDController
        );
        m_driveController.setTolerance(new Pose2d(Swerve.kGotoXYTolerance, Swerve.kGotoXYTolerance, new Rotation2d(Swerve.kGotoThetaTolerance)));

        m_currentField = new Field2d();
        m_targetField = new Field2d();

        m_targetCenterPoseField = new Field2d();

        m_fieldCentric = true;

        ArrayList<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> moduleConstants = new ArrayList<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>();
        moduleConstants.add(Constants.Swerve.FrontLeft);
        moduleConstants.add(Constants.Swerve.FrontRight);
        moduleConstants.add(Constants.Swerve.BackLeft);
        moduleConstants.add(Constants.Swerve.BackRight);

        m_mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
                Seconds.of(Constants.kSimLoopPeriod),
                Pounds.of(115),
                Inches.of(36),
                Inches.of(31),
                DCMotor.getKrakenX60(1),
                DCMotor.getKrakenX60(1),
                .8,
                getModuleLocations(),
                getPigeon2(),
                getModules(),
                moduleConstants
                );
    }

    @Override
    public void resetPose(Pose2d pose) {
        m_mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
    }

    public Pose2d currentPose() {
        return m_mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
    }

    public ChassisSpeeds getSpeed() {
        return m_mapleSimSwerveDrivetrain.mapleSimDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    }

    public Command resetRotation() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                getPigeon2().setYaw(0);
                m_mapleSimSwerveDrivetrain.getSimPigeon().setRawYaw(0);
            }).isFinished(true);
    }

    public Command increaseSpeed() {
        return new CommandBuilder(this).onExecute(() -> m_speedMultipler = Math.min(m_speedMultipler + Swerve.kSpeedStep, 1d)).isFinished(true);
    }

    public Command decreaseSpeed() {
        return new CommandBuilder(this).onExecute(() -> m_speedMultipler = Math.max(m_speedMultipler - Swerve.kSpeedStep, 0d)).isFinished(true);
    }

    public void setSpeedMultiplier(double speedMultipler) {
        m_speedMultipler = speedMultipler;
    }

    public Command pointWithController(DoubleSupplier leftX, DoubleSupplier leftY) {
        return CommandBuilder.none(this);
    }

    public Command driveLockedToArcWithJoysticks(DoubleSupplier leftX) {
        return driveToPose()
            .withFinishAllowance(false)
            .withTarget(
                () -> {
                    m_arcLockTheta += Math.toRadians(leftX.getAsDouble());

                    m_targetCenterPoseField.setRobotPose(m_arcLockCenter);

                    SmartDashboard.putData(m_targetCenterPoseField);

                    return new Pose2d(
                        Math.cos(m_arcLockTheta) * m_arcLockDistance + m_arcLockCenter.getX(),
                        Math.sin(m_arcLockTheta) * m_arcLockDistance + m_arcLockCenter.getY(),
                        new Rotation2d(m_arcLockTheta + Math.PI/2 - getShooterAngleCompensation())
                    );
                }
            )
            .onInitialize(
                () -> {
                    m_arcLockCenter = Helpers.allianceHub();

                    Pose2d currentPose = currentPose();

                    double dX = currentPose.getX() - m_arcLockCenter.getX();
                    double dY = currentPose.getY() - m_arcLockCenter.getY();

                    m_arcLockDistance = Math.hypot(dX, dY);
                    m_arcLockTheta = Math.atan2(dY, dX);
                }
            );
    }

    public Status status() {
        if (Broken.drivetrainFullyDisabled) return Status.DISABLED;
        if (!isCANSafe()) return Status.DISCONNECTED;
        if (m_isMoving) return Status.ACTIVE;
        return Status.IDLE;
    }

    public void setOptimalRotationGetter(DoubleSupplier supplier) {
        m_optimalRotationSupplier = supplier;
    }

    public Command alignToTowerY() {
        return driveToPose()
            .withTarget(
                () -> {
                    Pose2d targetPose = Helpers.getTargetHangPose(currentPose());

                    return new Pose2d(
                        targetPose.getX(),
                        currentPose().getY(),
                        targetPose.getRotation()
                    );
                }
            )
            .withVelocityPercentLimits(List.of(.25d, .2d, .25d));
    }

    public Command brick() {
        return CommandBuilder.none(this);
    }

    public Command idle() {
        return CommandBuilder.none(this);
    }

    public CommandBuilder driveToPose(Supplier<Pose2d> target) {
        return driveToPose().withTarget(target);
    }

    public DriveToPose driveToPose() {
        return new DriveToPose();
    }

    public CommandBuilder driveToPose(Supplier<Pose2d> target, List<Double> speedPercents) {
        return new DriveToPose().withTarget(target).withVelocityPercentLimits(speedPercents);
    }

    /**
     * <h3>DriveToPose</h3>
     * <p>This structure uses <code>with*</code> methods.</p>
     * <p>This means you can just create an instance with the helper method of <code>SwerveSubsystem.driveToPose</code> and only have to worry about adding the targetPose, along with anything else you need.</p>
     */
    public class DriveToPose extends CommandBuilder {
        private DoubleSupplier m_targetVelocity = () -> 0.0d;
        private BooleanSupplier m_allowedToFinish = () -> true;
        private double m_slowdownX = 1.0;
        private double m_slowdownY = 1.0;
        private double m_slowdownT = 1.0;

        public DriveToPose() {
            super(SimulatedSwerveSubsystem.this);
            isFinished(
                () -> m_driveController.atReference() && m_allowedToFinish.getAsBoolean()
            );
        }

        public DriveToPose withTargetVelocity(double target) {
            return withTargetVelocity(() -> target);
        }

        public DriveToPose withTargetVelocity(DoubleSupplier target) {
            m_targetVelocity = target;
            return this;
        }

        public DriveToPose withFinishAllowance(boolean allow) {
            return withFinishAllowance(() -> allow);
        }

        public DriveToPose withFinishAllowance(BooleanSupplier isAllowed) {
            m_allowedToFinish = isAllowed;
            isFinished(
                () -> m_driveController.atReference() && m_allowedToFinish.getAsBoolean()
            );
            return this;
        }

        public DriveToPose withTarget(Pose2d target) {
            return withTarget(() -> target);
        }

        public DriveToPose withVelocityPercentLimits(List<Double> percents) {
            if (percents.size() != 3) {
                Alert.critical("Percents array is of invalid length");
                return this;
            }
            m_slowdownX = percents.get(0);
            m_slowdownY = percents.get(1);
            m_slowdownT = percents.get(2);
            return this;
        }

        public DriveToPose withTarget(Supplier<Pose2d> target) {
            onExecute(() -> {
                    Pose2d targetPose = target.get();
                    ChassisSpeeds speeds = m_driveController.calculate(
                        currentPose(),
                        targetPose,
                        m_targetVelocity.getAsDouble(),
                        targetPose.getRotation()
                    );
                    setControl(
                        new SwerveRequest.RobotCentric()
                            .withVelocityX(speeds.vxMetersPerSecond * Swerve.kMaxSpeed * m_slowdownX)
                            .withVelocityY(speeds.vyMetersPerSecond * Swerve.kMaxSpeed * m_slowdownY)
                            .withRotationalRate(speeds.omegaRadiansPerSecond * Swerve.kMaxAngularRate * m_slowdownT)
                            .withDeadband(Swerve.kVelocityDeadband * 0.5)
                            .withRotationalDeadband(Swerve.kAngularVelocityDeadband)
                    );
                }
            );
            onInitialize(() -> {
                resetPose(currentPose());
            });
            return this;
        }
    }

    private boolean m_hasBeenRegistered = false;

    public void registerSubsystem() {
        if (!m_hasBeenRegistered) {
            CommandScheduler.getInstance().registerSubsystem(this);
            m_hasBeenRegistered = true;
        }
    }

    public boolean isRegistered() {
        return m_hasBeenRegistered;
    }

    public void setFieldCentric(boolean isOn) {
        m_fieldCentric = isOn;
    }

    public void setLimelightDisable(boolean isDisabled) {}

    @Override
    public void hddlPeriodic() {};

    public Command setHubLock(Boolean isOn) {
        return new CommandBuilder().onExecute(() -> {
            m_hubLock = isOn;
        }).isFinished(true);
    }

    public Command hubLock() {
        return new CommandBuilder()
            .onExecute(() -> {
                m_hubLock = true;
            })
            .onEnd(() -> {
                m_hubLock = false;
            });
    }

    public Command trenchLock() {
        return new CommandBuilder()
            .onExecute(() -> {
                Translation2d closestTrench = ZoneConstants.closestTrench(currentPose().getTranslation());

                System.out.println(closestTrench.getDistance(currentPose().getTranslation()));

                if (closestTrench.getDistance(currentPose().getTranslation()) > Constants.Swerve.kTrenchLockMaxDist) {
                    m_trenchLock = false;
                    return;
                }

                m_trenchYPos = closestTrench.getY();
                m_trenchLock = true;
            })
            .onEnd(() -> {
                m_trenchLock = false;
                return;
            });
    }

    public boolean isCANSafe() {
        if (Helpers.isBypassModeEnabled()) return true;
        for (int i = 0; i < getModules().length; i++) {
            if (!Helpers.onCANChain(getModule(i).getDriveMotor()) || !Helpers.onCANChain(getModule(i).getSteerMotor()) || !Helpers.onCANChain(getModule(i).getEncoder())) {
                return false;
            }
        }
        return true;
    }

    /**
     * {@code thetaSupplier} should be in degrees
     */
    public void ensureTheta(DoubleSupplier thetaSupplier) {
        m_ensureTheta = true;
        m_ensuredThetaSupplier = thetaSupplier;
    }

    public void clearEnsuredTheta() {
        m_ensureTheta = false;
        m_ensuredThetaSupplier = () -> 0;
    }

    public Command toggleFieldCentric() {
        return new CommandBuilder()
            .onExecute(() -> m_fieldCentric = !m_fieldCentric)
            .isFinished(true);
    }

    public void resetControl(Pose2d pose) {
        resetPose(pose);
        m_driveController.getXController().reset();
        m_driveController.getYController().reset();
        m_driveController.getThetaController().reset(pose.getRotation().getRadians());
    }

    public void setupTheta(boolean isAuto) {
        if (isAuto) {
            m_driveController.getThetaController().setPID(Constants.Swerve.ThetaAutoPID.kP, Constants.Swerve.ThetaAutoPID.kI, Constants.Swerve.ThetaAutoPID.kD);
        } else {
            m_driveController.getThetaController().setPID(Constants.Swerve.ThetaPID.kP, Constants.Swerve.ThetaPID.kI, Constants.Swerve.ThetaPID.kD);
        }
    }

    public double getShooterAngleCompensation() {
        Pose2d currentPose = currentPose();

        double dX = currentPose.getX() - HubOrchestrator.virtualHub.getX();
        double dY = currentPose.getY() - HubOrchestrator.virtualHub.getY();

        return Math.PI/2 - Math.acos(Constants.Swerve.kShooterOffset / Math.hypot(dX, dY));
    }

    public AbstractDriveTrainSimulation getAbstractSim() {
        return m_mapleSimSwerveDrivetrain.mapleSimDrive;
    }

    public Field2d getMainField() {
        return m_currentField;
    }
}