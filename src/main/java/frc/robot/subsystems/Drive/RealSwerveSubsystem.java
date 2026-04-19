package frc.robot.subsystems.Drive;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.orchestration.HubOrchestrator;
import frc.util.Alert;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants.Status;
import frc.util.Constants.Swerve;
import frc.util.LimelightHelpers.PoseEstimate;
import frc.util.LimelightHelpers.RawFiducial;
import frc.util.ZoneConstants.ZoneInfo;
import frc.util.LimelightHelpers;
import frc.util.ZoneConstants;
import frc.util.Constants;
import frc.util.Helpers;

import java.io.File;
import java.io.FileWriter;

public class RealSwerveSubsystem extends SwerveBase implements SwerveSubsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    private boolean m_fieldCentric;
    private final SwerveRequest.FieldCentric m_fieldCentricRequest = new SwerveRequest.FieldCentric();
    private final SwerveRequest.RobotCentric m_robotCentricRequest = new SwerveRequest.RobotCentric();
    private final SwerveRequest.SwerveDriveBrake m_brickRequest = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt m_pointRequest = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.Idle m_idleRequest = new SwerveRequest.Idle();
    private final SwerveRequest.ApplyRobotSpeeds m_autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    public SysID sysID;

    private HolonomicDriveController m_driveController;

    private Field2d m_currentField;
    private Field2d m_targetField;

    private Field2d m_rearLimelightEstimate;
    private Field2d m_mainLimelightEstimate;

    private Field2d m_targetCenterPoseField;

    private double m_arcLockDistance;
    private double m_arcLockTheta;
    
    private boolean m_hubLock = false;
    
    private boolean m_trenchLock = false;
    
    private double m_trenchYPos = 0;

    private boolean m_isMoving = false;

    private boolean m_limelightDisable;

    /** 0% - 100% of max speed */
    public double m_speedMultipler = 1.0; 

    private boolean m_ensureTheta = false;
    private DoubleSupplier m_ensuredThetaSupplier = () -> 0;

    File file = new File("/home/lvuser/logs/driverPreferences.csv");
    FileWriter outputfile;
    ZoneInfo lastZone = ZoneInfo.none();

    DoubleSupplier m_optimalRotationSupplier = () -> 0;

    public RealSwerveSubsystem() {
        super();

        sysID = new SysID(this);

        m_driveController = new HolonomicDriveController(
            Swerve.kHolonomicXPIDController,
            Swerve.kHolonomicYPIDController,
            Swerve.kHolonomicThetaPIDController
        );
        m_driveController.setTolerance(new Pose2d(Swerve.kGotoXYTolerance, Swerve.kGotoXYTolerance, new Rotation2d(Swerve.kGotoThetaTolerance)));

        m_currentField = new Field2d();
        m_targetField = new Field2d();

        m_rearLimelightEstimate = new Field2d();
        m_mainLimelightEstimate = new Field2d();

        m_targetCenterPoseField = new Field2d();

        m_fieldCentric = true;

        SmartDashboard.putData("Swerve Data", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Swerve Data");

                builder.addDoubleProperty("Module 0 Angle", () -> getModule(0).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Module 0 Vel", () -> getModule(0).getCurrentState().speedMetersPerSecond, null);
                builder.addDoubleProperty("Module 0 Steer Current", () -> getModule(0).getSteerMotor().getSupplyCurrent().getValueAsDouble(), null);
                builder.addDoubleProperty("Module 0 Drive Current", () -> getModule(0).getDriveMotor().getSupplyCurrent().getValueAsDouble(), null);
                builder.addDoubleProperty("Module 0 Steer Temp", () -> getModule(0).getSteerMotor().getDeviceTemp().getValueAsDouble(), null);
                builder.addDoubleProperty("Module 0 Drive Temp", () -> getModule(0).getDriveMotor().getDeviceTemp().getValueAsDouble(), null);

                builder.addDoubleProperty("Module 1 Angle", () -> getModule(1).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Module 1 Vel", () -> getModule(1).getCurrentState().speedMetersPerSecond, null);
                builder.addDoubleProperty("Module 1 Steer Current", () -> getModule(1).getSteerMotor().getSupplyCurrent().getValueAsDouble(), null);
                builder.addDoubleProperty("Module 1 Drive Current", () -> getModule(1).getDriveMotor().getSupplyCurrent().getValueAsDouble(), null);
                builder.addDoubleProperty("Module 1 Steer Temp", () -> getModule(1).getSteerMotor().getDeviceTemp().getValueAsDouble(), null);
                builder.addDoubleProperty("Module 1 Drive Temp", () -> getModule(1).getDriveMotor().getDeviceTemp().getValueAsDouble(), null);

                builder.addDoubleProperty("Module 2 Angle", () -> getModule(2).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Module 2 Vel", () -> getModule(2).getCurrentState().speedMetersPerSecond, null);
                builder.addDoubleProperty("Module 2 Steer Current", () -> getModule(2).getSteerMotor().getSupplyCurrent().getValueAsDouble(), null);
                builder.addDoubleProperty("Module 2 Drive Current", () -> getModule(2).getDriveMotor().getSupplyCurrent().getValueAsDouble(), null);
                builder.addDoubleProperty("Module 2 Steer Temp", () -> getModule(2).getSteerMotor().getDeviceTemp().getValueAsDouble(), null);
                builder.addDoubleProperty("Module 2 Drive Temp", () -> getModule(2).getDriveMotor().getDeviceTemp().getValueAsDouble(), null);

                builder.addDoubleProperty("Module 3 Angle", () -> getModule(3).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Module 3 Vel", () -> getModule(3).getCurrentState().speedMetersPerSecond, null);
                builder.addDoubleProperty("Module 3 Steer Current", () -> getModule(3).getSteerMotor().getSupplyCurrent().getValueAsDouble(), null);
                builder.addDoubleProperty("Module 3 Drive Current", () -> getModule(3).getDriveMotor().getSupplyCurrent().getValueAsDouble(), null);
                builder.addDoubleProperty("Module 3 Steer Temp", () -> getModule(3).getSteerMotor().getDeviceTemp().getValueAsDouble(), null);
                builder.addDoubleProperty("Module 3 Drive Temp", () -> getModule(3).getDriveMotor().getDeviceTemp().getValueAsDouble(), null);

                builder.addDoubleProperty("Robot Angle", () -> getPigeon2().getRotation2d().getRadians(), null);
            }
        });

        if (Utils.isSimulation()) {
            startSimThread();
        }

        configurePathPlanner();
    }

    @Override
    public void hddlPeriodic() {};

    public void setFieldCentric(boolean isOn) {
        m_fieldCentric = isOn;
    }

    public void setLimelightDisable(boolean isDisabled) {
        m_limelightDisable = isDisabled;
    }

    public Command toggleFieldCentric() {
        return new CommandBuilder()
            .onExecute(() -> m_fieldCentric = !m_fieldCentric)
            .isFinished(true);
    }

    public Command resetRotation() {
        return runOnce(this::seedFieldCentric);
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

    public Command driveWithJoysticks(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        return applyRequest(() -> {
            // YES! The y and x are swapped on purpose, it has to do with coordinate systems in the library so just leave it like this please!
            double actualSpeedMultiplier = m_speedMultipler;
            if (Helpers.isBypassModeEnabled()) actualSpeedMultiplier = 1d;

            double vx = -leftY.getAsDouble() * Constants.Swerve.kMaxSpeed * actualSpeedMultiplier;
            double vy = -leftX.getAsDouble() * Constants.Swerve.kMaxSpeed * actualSpeedMultiplier;
            double vRot = -rightX.getAsDouble() * Constants.Swerve.kMaxAngularRate * actualSpeedMultiplier;

            if (m_isTemporarilySlowed) {
                vx *= Constants.Swerve.kTemporarySlowdownAmount;
                vy *= Constants.Swerve.kTemporarySlowdownAmount;
                vRot *= Constants.Swerve.kTemporarySlowdownAmount;
            }

            if (m_hubLock) {
                Rotation2d targetAngle = new Rotation2d(m_optimalRotationSupplier.getAsDouble() - Math.PI/2 - getShooterAngleCompensation());

                vRot = m_driveController.calculate(
                        currentPose(),
                        new Pose2d(currentPose().getTranslation(), targetAngle),
                        0,
                        targetAngle
                    ).omegaRadiansPerSecond;
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
                return m_fieldCentricRequest
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(vRot)
                    .withDeadband(Constants.Swerve.kVelocityDeadband)
                    .withRotationalDeadband(Constants.Swerve.kAngularVelocityDeadband)
                    .withDesaturateWheelSpeeds(true);
            } else {
                return m_robotCentricRequest
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(vRot)
                    .withDeadband(Constants.Swerve.kVelocityDeadband)
                    .withRotationalDeadband(Constants.Swerve.kAngularVelocityDeadband)
                    .withDesaturateWheelSpeeds(true);
            }
        });
    }

    public Command driveWithVelocities(double vx, double vy, double vRot) {
        return applyRequest(() -> 
            m_robotCentricRequest
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(vRot)
                .withDeadband(Constants.Swerve.kVelocityDeadband)
                .withRotationalDeadband(Constants.Swerve.kAngularVelocityDeadband)
                .withDesaturateWheelSpeeds(true)
        );
    }

    public Command brick() {
        return applyRequest(() -> m_brickRequest).withName("driveBrick");
    }

    public Command pointWithController(DoubleSupplier leftX, DoubleSupplier leftY) {
        return applyRequest(() ->
            m_pointRequest.withModuleDirection(new Rotation2d(-leftY.getAsDouble(), -leftX.getAsDouble()))
        )
        .withName("drivePointWithController");
    }

    public Command idle() {
        return applyRequest(() -> m_idleRequest).ignoringDisable(true).withName("driveIdle");
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> {
            m_isMoving = false;
            if (isCANSafe() && !Broken.drivetrainFullyDisabled) {
                SwerveRequest req = request.get();
                m_isMoving = req.equals(m_idleRequest);
                this.setControl(request.get());
            } else {
                if (!Broken.drivetrainFullyDisabled)
                    Alert.critical("DRIVE DISABLED | CAN DISCONNECT");
                this.setControl(m_idleRequest);
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
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        if (!m_limelightDisable) {
            Matrix<N3, N1> teleStdDevs = new Matrix<N3, N1>(Nat.N3(), Nat.N1());
            teleStdDevs.set(0, 0, 0.1);
            teleStdDevs.set(1, 0, 0.1);
            teleStdDevs.set(2, 0, 99999999);

            Matrix<N3, N1> teleGoodStdDevs = new Matrix<N3, N1>(Nat.N3(), Nat.N1());
            teleGoodStdDevs.set(0, 0, 0.01);
            teleGoodStdDevs.set(1, 0, 0.01);
            teleGoodStdDevs.set(2, 0, 99999999);

            Matrix<N3, N1> wheelMeasurementStdDevs = new Matrix<N3, N1>(Nat.N3(), Nat.N1());
            wheelMeasurementStdDevs.set(0, 0, 0.01);
            wheelMeasurementStdDevs.set(1, 0, 0.01);
            wheelMeasurementStdDevs.set(2, 0, 0.01);

            setStateStdDevs(wheelMeasurementStdDevs);

            LimelightHelpers.SetRobotOrientation("limelight", currentPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.SetRobotOrientation("limelight-rear", currentPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

            LimelightHelpers.PoseEstimate mt2_shooter = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            LimelightHelpers.PoseEstimate mt2_rear = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-rear");
            if (mt2_shooter != null) {
                if (mt2_shooter.tagCount >= 2 ||
                    (mt2_shooter.tagCount == 1 && mt2_shooter.rawFiducials[0].ambiguity < 0.2)) {
                    double minimumDistance = mt2_shooter.rawFiducials[0].distToRobot;
                    for (RawFiducial fiducial : mt2_shooter.rawFiducials) {
                        if (fiducial.distToRobot < minimumDistance) minimumDistance = fiducial.distToRobot;
                    }
                    if (minimumDistance <= Constants.Swerve.kMaximumLimelightDistance || mt2_shooter.tagCount >= 2) {
                        if (mt2_shooter.tagCount > 2) {
                            setVisionMeasurementStdDevs(teleGoodStdDevs);
                        } else {
                            setVisionMeasurementStdDevs(teleStdDevs);
                        }
                        addVisionMeasurement(mt2_shooter.pose, mt2_shooter.timestampSeconds);
                    }
                }
            } else {
                Alert.warning("Couldn't find main limelight");
            }

            if (mt2_rear != null) {
                if (mt2_rear.tagCount >= 2 ||
                    (mt2_rear.tagCount == 1 && mt2_rear.rawFiducials[0].ambiguity < 0.2)) {
                    double minimumDistance = mt2_rear.rawFiducials[0].distToRobot;
                    for (RawFiducial fiducial : mt2_rear.rawFiducials) {
                        if (fiducial.distToRobot < minimumDistance) minimumDistance = fiducial.distToRobot;
                    }
                    if (minimumDistance <= Constants.Swerve.kMaximumLimelightDistance || mt2_rear.tagCount >= 2) {
                        if (mt2_rear.tagCount > 2) {
                            setVisionMeasurementStdDevs(teleGoodStdDevs);
                        } else {
                            setVisionMeasurementStdDevs(teleStdDevs);
                        }
                        addVisionMeasurement(mt2_rear.pose, mt2_rear.timestampSeconds);
                    }
                }
            } else {
                Alert.warning("Couldn't find rear limelight");
            }

            SmartDashboard.putNumber("SOTM / Drive Interpolated Theta", m_optimalRotationSupplier.getAsDouble());
            SmartDashboard.putNumber("SOTM / Drive Current Theta", currentPose().getRotation().getRadians());
        }

        m_currentField.setRobotPose(currentPose());
        SmartDashboard.putData("Drive / Current Pose", m_currentField);
        SmartDashboard.putData("Drive / Target Pose", m_targetField);
        if (!m_limelightDisable) {
            PoseEstimate rearLimelight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-rear");
            PoseEstimate limelight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            if (rearLimelight != null) m_rearLimelightEstimate.setRobotPose(rearLimelight.pose);
            if (limelight != null) m_mainLimelightEstimate.setRobotPose(limelight.pose);
            SmartDashboard.putData("Drive / Rear Limelight Estimate", m_rearLimelightEstimate);
            SmartDashboard.putData("Drive / Main Limelight Estimate", m_mainLimelightEstimate);
        }
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

        SmartDashboard.putNumber("Drive / Pigeon Rate RPS", getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        // Run simulation at a faster rate so PID gains behave more reasonably
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            // Use the measured time delta, get battery voltage from WPILib
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> StdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), StdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    public Pose2d currentPose() {
        return getState().Pose;
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
            super(RealSwerveSubsystem.this);
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

    public boolean isCANSafe() {
        if (Helpers.isBypassModeEnabled()) return true;
        for (int i = 0; i < getModules().length; i++) {
            if (!Helpers.onCANChain(getModule(i).getDriveMotor()) || !Helpers.onCANChain(getModule(i).getSteerMotor()) || !Helpers.onCANChain(getModule(i).getEncoder())) {
                return false;
            }
        }
        return true;
    }

    public Command driveLockedToArcWithJoysticks(DoubleSupplier leftX) {
        return driveToPose()
            .withFinishAllowance(false)
            .withTarget(
                () -> {
                    m_arcLockTheta += Math.toRadians(leftX.getAsDouble());

                    m_targetCenterPoseField.setRobotPose(Helpers.allianceHub());

                    SmartDashboard.putData(m_targetCenterPoseField);

                    return new Pose2d(
                        Math.cos(m_arcLockTheta) * m_arcLockDistance + Helpers.allianceHub().getX(),
                        Math.sin(m_arcLockTheta) * m_arcLockDistance + Helpers.allianceHub().getY(),
                        new Rotation2d(m_arcLockTheta + Math.PI/2 - getShooterAngleCompensation())
                    );
                }
            )
            .onInitialize(
                () -> {
                    Pose2d currentPose = currentPose();

                    double dX = currentPose.getX() - Helpers.allianceHub().getX();
                    double dY = currentPose.getY() - Helpers.allianceHub().getY();

                    m_arcLockDistance = Math.hypot(dX, dY);
                    m_arcLockTheta = Math.atan2(dY, dX);
                }
            );
    }

    public double getShooterAngleCompensation() {
        Pose2d currentPose = currentPose();

        double dX = currentPose.getX() - HubOrchestrator.virtualHub.getX();
        double dY = currentPose.getY() - HubOrchestrator.virtualHub.getY();

        return Math.PI/2 - Math.acos(Constants.Swerve.kShooterOffset / Math.hypot(dX, dY));
    }

    public void setupTheta(boolean isAuto) {
        if (isAuto) {
            m_driveController.getThetaController().setPID(Constants.Swerve.ThetaAutoPID.kP, Constants.Swerve.ThetaAutoPID.kI, Constants.Swerve.ThetaAutoPID.kD);
        } else {
            m_driveController.getThetaController().setPID(Constants.Swerve.ThetaPID.kP, Constants.Swerve.ThetaPID.kI, Constants.Swerve.ThetaPID.kD);
        }
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

                this.setControl(m_autoRequest.withSpeeds(speeds));
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

    public void resetControl(Pose2d pose) {
        resetPose(pose);
        m_driveController.getXController().reset();
        m_driveController.getYController().reset();
        m_driveController.getThetaController().reset(pose.getRotation().getRadians());
    }

    public Status status() {
        if (Broken.drivetrainFullyDisabled) return Status.DISABLED;
        if (!isCANSafe()) return Status.DISCONNECTED;
        if (m_isMoving) return Status.ACTIVE;
        return Status.IDLE;
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

    public ChassisSpeeds getSpeed() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getState().Speeds, currentPose().getRotation());
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

    private static boolean m_isTemporarilySlowed = false;
    public Command temporarySlowmode() {
        return new CommandBuilder() // should NOT require drivetrain 
            .onExecute(() -> {
                m_isTemporarilySlowed = true;
            })
            .onEnd(() -> {
                m_isTemporarilySlowed = false;
            })
            .isFinished(false);
    }
}
