package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Inches;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.orchestration.HubOrchestrator;
import frc.util.Alert;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Constants.Status;
import frc.util.Constants.Swerve;
import frc.util.Helpers;
import frc.util.ZoneConstants;

public class SimulatedSwerveSubsystem implements SwerveSubsystem {
    private SelfControlledSwerveDriveSimulation m_mapleSimDrivetrain;
    private final Field2d m_currentField;

    // private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    // private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    // private boolean m_hasAppliedOperatorPerspective = false;

    private boolean m_fieldCentric;

    public SysID sysID;

    private HolonomicDriveController m_driveController;

    private Field2d m_targetField;

    // private Field2d m_targetCenterPoseField;

    // private Pose2d m_arcLockCenter;
    // private double m_arcLockDistance;
    // private double m_arcLockTheta;
    
    private boolean m_hubLock = false;
    
    private boolean m_trenchLock = false;
    
    private double m_trenchYPos = 0;

    private boolean m_isMoving = false;

    /** 0% - 100% of max speed */
    public double m_speedMultipler = 1.0; 

    private boolean m_ensureTheta = false;
    private DoubleSupplier m_ensuredThetaSupplier = () -> 0;

    private DoubleSupplier m_optimalRotationSupplier = () -> 0;

    public SimulatedSwerveSubsystem() {
        DriveTrainSimulationConfig mapleSimDrivetrainConfig = DriveTrainSimulationConfig.Default()
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                DCMotor.getKrakenX60(4), 
                DCMotor.getKrakenX60(4), 
                Constants.Swerve.kDriveGearRatio,
                Constants.Swerve.kSteerGearRatio,
                Constants.Swerve.kDriveFrictionVoltage,
                Constants.Swerve.kSteerFrictionVoltage,
                Constants.Swerve.kWheelRadius,
                Constants.Swerve.kSteerInertia,
                Constants.Swerve.kWheelCoefficientOfFriction)
            )
            .withTrackLengthTrackWidth(Inches.of(9.875 * 2), Inches.of(12.5 * 2))
            .withBumperSize(Inches.of(31), Inches.of(31));

        m_mapleSimDrivetrain = new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(mapleSimDrivetrainConfig, Pose2d.kZero));
        SimulatedArena.getInstance().addDriveTrainSimulation(m_mapleSimDrivetrain.getDriveTrainSimulation());

        m_currentField = new Field2d();
        SmartDashboard.putData("Drive / Current Pose", m_currentField);

        m_driveController = new HolonomicDriveController(
            Swerve.kHolonomicXPIDController,
            Swerve.kHolonomicYPIDController,
            Swerve.kHolonomicThetaPIDController
        );
        m_driveController.setTolerance(new Pose2d(Swerve.kGotoXYTolerance, Swerve.kGotoXYTolerance, new Rotation2d(Swerve.kGotoThetaTolerance)));

        m_fieldCentric = true;

        configurePathPlanner();
    }

    public Command driveWithJoysticks(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        return new CommandBuilder(this).onExecute(() -> {
            // YES! The y and x are swapped on purpose, it has to do with coordinate systems in the library so just leave it like this please!
            double actualSpeedMultiplier = m_speedMultipler;
            if (Helpers.isBypassModeEnabled()) actualSpeedMultiplier = 1d;

            double vx = -leftY.getAsDouble() * Constants.Swerve.kMaxSpeed * actualSpeedMultiplier;
            double vy = -leftX.getAsDouble() * Constants.Swerve.kMaxSpeed * actualSpeedMultiplier;
            double vRot = -rightX.getAsDouble() * Constants.Swerve.kMaxAngularRate * actualSpeedMultiplier;

            if (m_hubLock) {
                // m_arcLockCenter = Helpers.allianceHub();

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

            m_mapleSimDrivetrain.runChassisSpeeds(new ChassisSpeeds(vx, vy, vRot), new Translation2d(), m_fieldCentric, true);
        });
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return CommandBuilder.none(this);
    }

    @Override
    public ChassisSpeeds getSpeed() {
        return m_mapleSimDrivetrain.getMeasuredSpeedsRobotRelative(true);
    }

    @Override
    public Pose2d currentPose() {
        return m_mapleSimDrivetrain.getOdometryEstimatedPose();
    }

    @Override
    public void resetControl(Pose2d pose) {
        m_mapleSimDrivetrain.setSimulationWorldPose(pose);
        m_mapleSimDrivetrain.resetOdometry(pose);
        m_driveController.getXController().reset();
        m_driveController.getYController().reset();
        m_driveController.getThetaController().reset(pose.getRotation().getRadians());
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        // m_mapleSimDrivetrain.addVisionEstimation(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> StdDevs
    ) {
        // m_mapleSimDrivetrain.addVisionEstimation(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), StdDevs);
    }

    @Override
    public void periodic() {
        m_mapleSimDrivetrain.periodic();
        // if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
        //     DriverStation.getAlliance().ifPresent(allianceColor -> {
        //         setOperatorPerspectiveForward(
        //             allianceColor == Alliance.Red
        //                 ? kRedAlliancePerspectiveRotation
        //                 : kBlueAlliancePerspectiveRotation
        //         );
        //         m_hasAppliedOperatorPerspective = true;
        //     });
        // }

        SmartDashboard.putNumber("SOTM / Drive Interpolated Theta", m_optimalRotationSupplier.getAsDouble());
        SmartDashboard.putNumber("SOTM / Drive Current Theta", currentPose().getRotation().getRadians());

        // SmartDashboard.putData("Drive / Target Pose", m_targetField);
        SmartDashboard.putNumber("Drive / Speed Multiplier", m_speedMultipler);

        SmartDashboard.putString("Drive / Current Zone", ZoneConstants.checkZone(currentPose().getTranslation()).fullName());

        SmartDashboard.putString("Drive / Robot Drive Mode", m_fieldCentric ? "Field Centric" : "Robot Centric");
        SmartDashboard.putNumber("Drive / Trench Y M", m_trenchYPos);
        SmartDashboard.putBoolean("Drive / Trench Lock", m_trenchLock);
        SmartDashboard.putBoolean("Drive / Hub Lock", m_hubLock);

        SmartDashboard.putData(m_driveController.getXController());
        SmartDashboard.putData(m_driveController.getYController());
        SmartDashboard.putData(m_driveController.getThetaController());
        SmartDashboard.putNumber("Drive / Theta Goal DEG", m_driveController.getThetaController().getGoal().position / Math.PI * 180);
        SmartDashboard.putNumber("Drive / Theta Setpoint DEG", m_driveController.getThetaController().getSetpoint().position / Math.PI * 180);

        m_currentField.setRobotPose(m_mapleSimDrivetrain.getActualPoseInSimulationWorld());
        m_currentField.getObject("odometry").setPose(currentPose());
        SmartDashboard.putData("Drive / Current Pose", m_currentField);
    }

    @Override
    public void hddlPeriodic() {};

    public void setSpeedMultiplier(double speedMultipler) {
        m_speedMultipler = speedMultipler;
    }

    public void setFieldCentric(boolean isOn) {
        m_fieldCentric = isOn;
    }

    public void setLimelightDisable(boolean isDisabled) {}

    public Command increaseSpeed() {
        return new CommandBuilder(this).onExecute(() -> m_speedMultipler = Math.min(m_speedMultipler + Swerve.kSpeedStep, 1d)).isFinished(true);
    }
    
    public Command decreaseSpeed() {
        return new CommandBuilder(this).onExecute(() -> m_speedMultipler = Math.max(m_speedMultipler - Swerve.kSpeedStep, 0d)).isFinished(true);
    }

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

    public Command brick() {
        return CommandBuilder.none(this);//applyRequest(() -> m_brickRequest).withName("driveBrick");
    }

    public Command pointWithController(DoubleSupplier leftX, DoubleSupplier leftY) {
        return CommandBuilder.none(this);//return applyRequest(() ->
        //     m_pointRequest.withModuleDirection(new Rotation2d(-leftY.getAsDouble(), -leftX.getAsDouble()))
        // )
        // .withName("drivePointWithController");
    }

    public Command idle() {
        return CommandBuilder.none(this);//return applyRequest(() -> m_idleRequest).ignoringDisable(true).withName("driveIdle");
    }

    public Command toggleFieldCentric() {
        return new CommandBuilder()
            .onExecute(() -> m_fieldCentric = !m_fieldCentric)
            .isFinished(true);
    }

    public Command resetRotation() {
        return runOnce(this::seedFieldCentric);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return Optional.of(new Pose2d());
    }

    public boolean isCANSafe() {
        return true;
    }

    public Command driveLockedToArcWithJoysticks(DoubleSupplier leftX) {
        return CommandBuilder.none(this);
    }

    @Override
    public Status status() {
        if (Broken.drivetrainFullyDisabled) return Status.DISABLED;
        if (!isCANSafe()) return Status.DISCONNECTED;
        if (m_isMoving) return Status.ACTIVE;
        return Status.IDLE;
    }

    @Override
    public void seedFieldCentric() {}

    public void ensureTheta(DoubleSupplier thetaSupplier) {
        m_ensureTheta = true;
        m_ensuredThetaSupplier = thetaSupplier;
    }

    public void clearEnsuredTheta() {
        m_ensureTheta = false;
        m_ensuredThetaSupplier = () -> 0;
    }

    public void setOptimalRotationGetter(DoubleSupplier supplier) {
        m_optimalRotationSupplier = supplier;
    }


    public void setupTheta(boolean isAuto) {
        if (isAuto) {
            m_driveController.getThetaController().setPID(Constants.Swerve.ThetaAutoPID.kP, Constants.Swerve.ThetaAutoPID.kI, Constants.Swerve.ThetaAutoPID.kD);
        } else {
            m_driveController.getThetaController().setPID(Constants.Swerve.ThetaPID.kP, Constants.Swerve.ThetaPID.kI, Constants.Swerve.ThetaPID.kD);
        }
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
        // private double m_slowdownX = 1.0;
        // private double m_slowdownY = 1.0;
        // private double m_slowdownT = 1.0;

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
            // m_slowdownX = percents.get(0);
            // m_slowdownY = percents.get(1);
            // m_slowdownT = percents.get(2);
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
                    m_mapleSimDrivetrain.runChassisSpeeds(speeds, Translation2d.kZero, false, true);
                }
            );
            onInitialize(() -> {
                resetControl(currentPose());
            });
            return this;
        }
    }

    public double getShooterAngleCompensation() {
        Pose2d currentPose = currentPose();

        double dX = currentPose.getX() - HubOrchestrator.virtualHub.getX();
        double dY = currentPose.getY() - HubOrchestrator.virtualHub.getY();

        return Math.PI/2 - Math.acos(Constants.Swerve.kShooterOffset / Math.hypot(dX, dY));
    }

    public Rectangle2d getNearestTrench() {
        Translation2d current = currentPose().getTranslation();

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            double leftDistance = ZoneConstants.Zones.kLeftBlueTrench.getDistance(current);
            double rightDistance = ZoneConstants.Zones.kRightBlueTrench.getDistance(current);
            if (leftDistance < rightDistance) return ZoneConstants.Zones.kLeftBlueTrench;
            return ZoneConstants.Zones.kRightBlueTrench;
        } else {
            double leftDistance = ZoneConstants.Zones.kLeftRedTrench.getDistance(current);
            double rightDistance = ZoneConstants.Zones.kRightRedTrench.getDistance(current);
            if (leftDistance < rightDistance) return ZoneConstants.Zones.kLeftRedTrench;
            return ZoneConstants.Zones.kRightRedTrench;
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

    private void configurePathPlanner() {
        RobotConfig autonomousConfig = null;
        try {
            autonomousConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (AutoBuilder.isConfigured()) return;

        AutoBuilder.configure(
            this::currentPose,
            this::resetControl,
            this::getSpeed,
            (speeds, feeds) -> {
                SmartDashboard.putNumber("Drive / Auto Speeds / X M", speeds.vxMetersPerSecond);
                SmartDashboard.putNumber("Drive / Auto Speeds / Y M", speeds.vyMetersPerSecond);
                SmartDashboard.putNumber("Drive / Auto Speeds / Theta RPS", speeds.omegaRadiansPerSecond);

                m_mapleSimDrivetrain.runChassisSpeeds(speeds, Translation2d.kZero, m_fieldCentric, true);
            },
            new PPHolonomicDriveController(
                new PIDConstants(Constants.Swerve.XYPID.kP, Constants.Swerve.XYPID.kI, Constants.Swerve.XYPID.kD),
                new PIDConstants(Constants.Swerve.ThetaAutoPID.kP, Constants.Swerve.ThetaAutoPID.kI, Constants.Swerve.ThetaAutoPID.kD)
            ),
            autonomousConfig, 
            () -> !Helpers.isBlueAlliance(),
            this
        );

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }
}
