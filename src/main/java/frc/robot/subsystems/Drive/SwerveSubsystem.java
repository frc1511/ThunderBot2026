package frc.robot.subsystems.Drive;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.util.CommandBuilder;
import frc.util.Constants.SwerveConstants;
import frc.util.LimelightHelpers;
import frc.util.Constants;

public class SwerveSubsystem extends SwerveBase implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    private boolean m_fieldCentric;
    private SwerveRequest.FieldCentric m_fieldCentricRequest = new SwerveRequest.FieldCentric();
    private SwerveRequest.RobotCentric m_robotCentricRequest = new SwerveRequest.RobotCentric();

    public SysID sysID;

    private HolonomicDriveController driveController;

    private boolean m_cancelGoto = false;

    private Field2d m_currentField;

    public SwerveSubsystem() {
        super();

        sysID = new SysID(this);

        driveController = new HolonomicDriveController(
            SwerveConstants.kHolonomicXPIDController,
            SwerveConstants.kHolonomicYPIDController,
            SwerveConstants.kHolonomicThetaPIDController
        );
        driveController.setTolerance(new Pose2d(SwerveConstants.kGotoXYTolerance, SwerveConstants.kGotoXYTolerance, new Rotation2d(SwerveConstants.kGotoThetaTolerance)));

        m_currentField = new Field2d();

        m_fieldCentric = true;

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command toggleFieldCentric() {
        return new CommandBuilder()
            .onExecute(() -> {
                m_fieldCentric = !m_fieldCentric;
            })
            .isFinished(() -> true);
    }

    public Command driveWithJoysticks(double leftX, double leftY, double rightX) {
        System.out.println(leftX);
        if (m_fieldCentric) {
            return applyRequest(() -> m_fieldCentricRequest
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(-leftY * Constants.SwerveConstants.kMaxSpeed)
                .withVelocityY(-leftX * Constants.SwerveConstants.kMaxSpeed)
                .withRotationalRate(-rightX * Constants.SwerveConstants.kMaxAngularRate)
                .withDeadband(.4 * Constants.SwerveConstants.kMaxSpeed)
                .withRotationalDeadband(.1 * Constants.SwerveConstants.kMaxAngularRate)
            );
        } else {
            return applyRequest(() -> m_robotCentricRequest
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(-leftY * Constants.SwerveConstants.kMaxSpeed)
                .withVelocityY(-leftX * Constants.SwerveConstants.kMaxSpeed)
                .withRotationalRate(-rightX * Constants.SwerveConstants.kMaxAngularRate)
                .withDeadband(.4 * Constants.SwerveConstants.kMaxSpeed)
                .withRotationalDeadband(.1 * Constants.SwerveConstants.kMaxAngularRate)
            );
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
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

        // LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        // addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);

        m_currentField.setRobotPose(currentPose());
        SmartDashboard.putData("currentPose", m_currentField);
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

    private Pose2d currentPose() {
        return getState().Pose;
    }

    public Command driveToPose(Pose2d targetPose, Rotation2d targetRotation, double targetVelocity) {
        return new CommandBuilder(this)
            .onExecute(
                () -> {
                    ChassisSpeeds speeds = driveController.calculate(
                        currentPose(),
                        targetPose,
                        targetVelocity,
                        targetRotation
                    );
                    Field2d targetField = new Field2d();
                    targetField.setRobotPose(targetPose);
                    SmartDashboard.putData("targetPose", targetField);
                    SmartDashboard.putNumber("targetVelX", speeds.vxMetersPerSecond * SwerveConstants.kMaxSpeed);
                    SmartDashboard.putNumber("targetVelY", speeds.vyMetersPerSecond * SwerveConstants.kMaxSpeed);
                    SmartDashboard.putNumber("targetVelTheta", speeds.omegaRadiansPerSecond * SwerveConstants.kMaxAngularRate);
                    SmartDashboard.putNumber("currentVelTheta", getModule(0).getSteerMotor().getVelocity().getValueAsDouble());
                    SmartDashboard.putNumber("currentTheta", currentPose().getRotation().getDegrees());
                    SmartDashboard.putNumber("targetTheta", targetRotation.getDegrees());
                    setControl(
                        new SwerveRequest.RobotCentric()
                            .withVelocityX(speeds.vxMetersPerSecond * SwerveConstants.kMaxSpeed)
                            .withVelocityY(speeds.vyMetersPerSecond * SwerveConstants.kMaxSpeed)
                            .withRotationalRate(speeds.omegaRadiansPerSecond * SwerveConstants.kMaxAngularRate)
                    );
                }
            )
            .isFinished(
                () -> {
                    if (m_cancelGoto) {
                        m_cancelGoto = false;
                        return true;
                    }
                    return driveController.atReference() || m_cancelGoto;
                }
            )
            .withName("driveToPose");
    }

    public boolean isCANSafe() {
        for (int i = 0; i < getModules().length; i++) {
            if (!getModule(i).getDriveMotor().isConnected(Constants.kCANChainDisconectTimout) || !getModule(i).getSteerMotor().isConnected(Constants.kCANChainDisconectTimout)) {
                System.err.println("[!!!!!!!] THERE IS A BIG O PROBLEM WITH THE DRIVE TRAIN D: (CAN DISCONECT)");
                return false;
            }
        }
        return true;
    }
}
