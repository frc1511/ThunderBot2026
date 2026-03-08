package frc.robot.subsystems.Drive;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.thunder.lib.trajectory.ThunderTrajectoryRunnerProperties;

import edu.wpi.first.math.Matrix;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.util.Alert;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants.Status;
import frc.util.Constants.Swerve;
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

    public SysID sysID;

    private HolonomicDriveController m_driveController;

    private Field2d m_currentField;
    private Field2d m_targetField;

    private Field2d m_targetCenterPoseField;

    private Pose2d m_arcLockCenter;
    private double m_arcLockDistance;
    private double m_arcLockTheta;
    
    private boolean m_hubLock = false;
    
    private boolean m_trenchLock = false;
    
    private double m_trenchXPos = 0;

    private boolean m_isMoving = false;

    private boolean m_limelightDisable;

    public double m_speedMultipler = 1.0; // 0% - 100% of max speed

    private boolean m_ensureTheta = false;
    private DoubleSupplier m_ensuredThetaSupplier = () -> 0;

    private Pose2d m_lastPose;
    private Pose2d m_currentPose;

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

        m_targetCenterPoseField = new Field2d();

        m_fieldCentric = true;

        m_lastPose = Pose2d.kZero;
        m_currentPose = Pose2d.kZero;

        SmartDashboard.putData("Swerve Data", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Swerve Data");

                builder.addDoubleProperty("Module 0 Angle", () -> getModule(0).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Module 0 Vel", () -> getModule(0).getCurrentState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Module 1 Angle", () -> getModule(1).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Module 1 Vel", () -> getModule(1).getCurrentState().speedMetersPerSecond, null);
                
                builder.addDoubleProperty("Module 2 Angle", () -> getModule(2).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Module 2 Vel", () -> getModule(2).getCurrentState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Module 3 Angle", () -> getModule(3).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Module 3 Vel", () -> getModule(3).getCurrentState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", () -> getPigeon2().getRotation2d().getRadians(), null);
            }
        });

        if (Utils.isSimulation()) {
            startSimThread();
        }

        // try {
            // boolean needsInit = !file.exists();
            // FileWriter outputfile = new FileWriter(file);
            
            // if (needsInit) {
            //     CSVWriter writer = new CSVWriter(outputfile);
    
            //     String[] header = {"Area", "Orientation"};
            //     writer.writeNext(header);
    
            //     writer.close();
            // }
        // }
        // catch (IOException e) {
        //     e.printStackTrace();
        // }
    }

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

    public Command increaseSpeed() {
        return new InstantCommand(() -> m_speedMultipler = Math.min(m_speedMultipler + Swerve.kSpeedStep, 1d));
    }
    
    public Command decreaseSpeed() {
        return new InstantCommand(() -> m_speedMultipler = Math.max(m_speedMultipler - Swerve.kSpeedStep, 0d));
    }

    public void setSpeedMultiplier(double speedMultipler) {
        m_speedMultipler = speedMultipler;
    }

    public Command setHubLock(Boolean isOn) {
        return new CommandBuilder().onExecute(() -> {
            m_hubLock = isOn;
        });
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

    public Command toggleTrenchLock() {
        return new CommandBuilder()
            .onExecute(() -> {
                if (m_trenchLock) {
                    m_trenchLock = false;
                    return;
                }
                Translation2d closestTrench = ZoneConstants.closestTrench(currentPose().getTranslation());

                if (closestTrench.getDistance(currentPose().getTranslation()) > Constants.Swerve.kTrenchLockMaxDist) return;

                m_trenchXPos = closestTrench.getX();
                m_trenchLock = true;
            })
            .isFinished(true);
    }

    public Command driveWithJoysticks(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        return applyRequest(() -> {
            // YES! The y and x are swapped on purpose, it has to do with coordinate systems in the library so just leave it like this please!
            double vx = -leftY.getAsDouble() * Constants.Swerve.kMaxSpeed * m_speedMultipler;
            double vy = -leftX.getAsDouble() * Constants.Swerve.kMaxSpeed * m_speedMultipler;
            double vRot = -rightX.getAsDouble() * Constants.Swerve.kMaxAngularRate * m_speedMultipler;

            if (m_hubLock) {
                m_arcLockCenter = Helpers.allianceHub();

                double dX = currentPose().getX() - m_arcLockCenter.getX();
                double dY = currentPose().getY() - m_arcLockCenter.getY();

                Rotation2d targetAngle = new Rotation2d(Math.atan2(dY, dX) + Math.PI/2 - getShooterAngleCompensation());

                SmartDashboard.putNumber("Drive intended hub lock theta", m_optimalRotationSupplier.getAsDouble());

                vRot = m_driveController.calculate(
                        currentPose(),
                        new Pose2d(currentPose().getTranslation(), targetAngle),
                        Math.hypot(dX, dY),
                        targetAngle
                    ).omegaRadiansPerSecond * Swerve.kMaxAngularRate;
            }

            if (m_trenchLock) {
                Translation2d currentTranslation = currentPose().getTranslation();
                
                vx = m_driveController.calculate(
                        currentPose(),
                        new Pose2d(new Translation2d(m_trenchXPos, currentTranslation.getY()), currentPose().getRotation()),
                        0,
                        currentPose().getRotation()
                    ).vxMetersPerSecond * Swerve.kMaxSpeed;
            }

            if (m_ensureTheta) {
                vRot = m_driveController.getThetaController().calculate(currentPose().getRotation().getDegrees(), m_ensuredThetaSupplier.getAsDouble());
            }

            if (DriverStation.isAutonomous()) {
                vx = m_autoXSupplier.getAsDouble();
                vy = m_autoYSupplier.getAsDouble();
                vRot = m_autoTSupplier.getAsDouble();
            }

            if (m_fieldCentric && !DriverStation.isAutonomous()) {
                return m_fieldCentricRequest
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(vRot)
                    .withDeadband(Constants.Swerve.kVelocityDeadband)
                    .withRotationalDeadband(Constants.Swerve.kAngularVelocityDeadband);
            } else {
                return m_robotCentricRequest
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(vRot)
                    .withDeadband(Constants.Swerve.kVelocityDeadband)
                    .withRotationalDeadband(Constants.Swerve.kAngularVelocityDeadband);
            }
        });
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
            LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
            LimelightHelpers.PoseEstimate limelightRearMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-rear");
            if (limelightMeasurement != null) {
                if (limelightMeasurement.tagCount >= 2 ||
                    (limelightMeasurement.tagCount == 1 && limelightMeasurement.rawFiducials[0].ambiguity < 0.2)) {
                    addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
                }
            } else {
                Alert.warning("Couldn't find main limelight");
            }

            if (limelightRearMeasurement != null) {
                if (limelightRearMeasurement.tagCount >= 2 ||
                    (limelightRearMeasurement.tagCount == 1 && limelightRearMeasurement.rawFiducials[0].ambiguity < 0.2)) {
                    addVisionMeasurement(limelightRearMeasurement.pose, limelightRearMeasurement.timestampSeconds);
                }
            } else {
                Alert.warning("Couldn't find rear limelight");
            }
        }

        // Update poses for bump detection
        if (m_currentPose != null) {
            m_lastPose = m_currentPose;
        }
        
        m_currentPose = currentPose();
        
        // Write to log

        // String[] line = {null, String.valueOf(m_currentPose.getRotation().getDegrees())};

        ZoneInfo currentZone = ZoneConstants.checkZone(m_currentPose.getTranslation());

        // SmartDashboard.putData("currentPose", m_currentField);

        // if (currentZone != lastZone && currentZone.isWithinOne) {
        //     line[0] = currentZone.shortName();
        //     try {
        //         CSVWriter writer = new CSVWriter(outputfile);
                
        //         writer.writeNext(line);
        //         writer.close();
        //     }
        //     catch (IOException e) {
        //         e.printStackTrace();
        //     }
        // }

        m_currentField.setRobotPose(currentPose());
        SmartDashboard.putData("currentPose", m_currentField);
        SmartDashboard.putData("targetPose", m_targetField);
        SmartDashboard.putNumber("Drive_speedMultiplier", m_speedMultipler);

        SmartDashboard.putString("currentZone", currentZone.fullName());

        SmartDashboard.putString("Robot drive mode", m_fieldCentric ? "Field Centric" : "Robot Centric");
        SmartDashboard.putNumber("trenchX", m_trenchXPos);

        SmartDashboard.putData(m_driveController.getXController());
        SmartDashboard.putData(m_driveController.getYController());
        SmartDashboard.putData(m_driveController.getThetaController());
        SmartDashboard.putNumber("Drive_theta_target_deg", m_driveController.getThetaController().getGoal().position / Math.PI * 180);

        SmartDashboard.putNumber("Drive_auto_vx", m_autoXSupplier.getAsDouble());
        SmartDashboard.putNumber("Drive_auto_vy", m_autoYSupplier.getAsDouble());
        SmartDashboard.putNumber("Drive_auto_vtheta", m_autoTSupplier.getAsDouble());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
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
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    public Pose2d currentPose() {
        return getState().Pose;
    }

    public DriveToPose driveToPose() {
        return new DriveToPose();
    }

    /**
     * <h3>DriveToPose</h3>
     * <p>This structure uses <code>with*</code> methods.</p>
     * <p>This means you can just create an instance with the helper method of <code>SwerveSubsystem.driveToPose</code> and only have to worry about adding the targetPose, along with anything else you need.</p>
     */
    public class DriveToPose extends CommandBuilder {
        private DoubleSupplier m_targetVelocity = () -> 0.0d;
        private BooleanSupplier m_allowedToFinish = () -> true;

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

        public DriveToPose withTarget(Supplier<Pose2d> target) {
            onExecute(() -> {
                    Pose2d targetPose = target.get();
                    ChassisSpeeds speeds = m_driveController.calculate(
                        currentPose(),
                        targetPose,
                        m_targetVelocity.getAsDouble(),
                        targetPose.getRotation()
                    );
                    m_targetField.setRobotPose(targetPose);
                    setControl(
                        new SwerveRequest.RobotCentric()
                            .withVelocityX(speeds.vxMetersPerSecond * Swerve.kMaxSpeed)
                            .withVelocityY(speeds.vyMetersPerSecond * Swerve.kMaxSpeed)
                            .withRotationalRate(speeds.omegaRadiansPerSecond * Swerve.kMaxAngularRate)
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

    private double getShooterAngleCompensation() {
        Pose2d currentPose = currentPose();

        double dX = currentPose.getX() - m_arcLockCenter.getX();
        double dY = currentPose.getY() - m_arcLockCenter.getY();

        return Math.PI/2 - Math.acos(Constants.Swerve.kShooterOffset / Math.hypot(dX, dY));
    }

    private DoubleSupplier m_autoXSupplier = () -> 0;
    private DoubleSupplier m_autoYSupplier = () -> 0;
    private DoubleSupplier m_autoTSupplier = () -> 0;
    private void setSpeeds(ChassisSpeeds speeds) {
        SmartDashboard.putNumber("SetSpeeds_x", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("SetSpeeds_y", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("SetSpeeds_theta", speeds.omegaRadiansPerSecond);

        double vRot = speeds.omegaRadiansPerSecond;

        if (m_hubLock) {
            m_arcLockCenter = Helpers.allianceHub();

            double dX = currentPose().getX() - m_arcLockCenter.getX();
            double dY = currentPose().getY() - m_arcLockCenter.getY();
            
            Rotation2d targetAngle = new Rotation2d(Math.atan2(dY, dX) + Math.PI/2 - getShooterAngleCompensation());

            System.out.println(m_optimalRotationSupplier.getAsDouble());

            vRot = m_driveController.calculate(
                currentPose(),
                new Pose2d(currentPose().getTranslation(), targetAngle),
                Math.hypot(dX, dY),
                targetAngle
            ).omegaRadiansPerSecond * Swerve.kMaxAngularRate;
        }

        final double vRotFin = vRot;

        m_autoXSupplier = () -> speeds.vxMetersPerSecond;
        m_autoYSupplier = () -> speeds.vyMetersPerSecond;
        m_autoTSupplier = () -> vRotFin;
    }

    public void resetControl(Pose2d pose) {
        resetPose(pose);
        m_driveController.getXController().reset();
        m_driveController.getYController().reset();
        m_driveController.getThetaController().reset(pose.getRotation().getRadians());
    }

    public ThunderTrajectoryRunnerProperties getTrajectoryRunnerProperties() {
        return new ThunderTrajectoryRunnerProperties(
            this::currentPose,
            pose -> resetControl(pose),
            speeds -> setSpeeds(speeds),
            m_driveController
        );
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
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
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

    // public Command alignToTowerY() {
    //     return driveToPose()
            // .withTarget(new Pose2d(currentPose().getX(), DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Constants.HangConstants.kTowerDistanceFromWallY : AprilTagFields.k2026RebuiltWelded.))
    // }
}
