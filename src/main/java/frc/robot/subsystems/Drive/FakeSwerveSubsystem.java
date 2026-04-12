package frc.robot.subsystems.Drive;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.util.CommandBuilder;
import frc.util.Constants.Status;

public class FakeSwerveSubsystem implements SwerveSubsystem {
    public FakeSwerveSubsystem() {}

    @Override
    public void hddlPeriodic() {};

    public void setSpeedMultiplier(double speedMultipler) {}

    public void setFieldCentric(boolean isOn) {}

    public void setLimelightDisable(boolean isDisabled) {}

    public Command increaseSpeed() {
        return CommandBuilder.none(this);
    }
    
    public Command decreaseSpeed() {
        return CommandBuilder.none(this);
    }

    public Command setHubLock(Boolean isOn) {
        return CommandBuilder.none(this);
    }

    public Command hubLock() {
        return CommandBuilder.none(this);
    }

    public Command trenchLock() {
        return CommandBuilder.none(this);
    }

    public Command driveWithJoysticks(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        return CommandBuilder.none(this);
    }

    public Command brick() {
        return CommandBuilder.none(this);
    }

    public Command pointWithController(DoubleSupplier leftX, DoubleSupplier leftY) {
        return CommandBuilder.none(this);
    }

    public Command idle() {
        return CommandBuilder.none(this);
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return CommandBuilder.none(this);
    }

    public Command toggleFieldCentric() {
        return CommandBuilder.none(this);
    }
    
    public Command resetRotation() {
        return CommandBuilder.none(this);
    }

    @Override
    public void periodic() {}
    
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {}

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {}

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return Optional.of(new Pose2d());
    }

    public Pose2d currentPose() {
        return new Pose2d();
    }

    public boolean isCANSafe() {
        return false;
    }

    public Command driveLockedToArcWithJoysticks(DoubleSupplier leftX) {
        return CommandBuilder.none(this);
    }

    public void resetControl(Pose2d pose) {}

    @Override
    public Status status() {
        return Status.DISABLED;
    }

    @Override
    public void seedFieldCentric() {}

    public void ensureTheta(DoubleSupplier thetaSupplier) {}

    public void clearEnsuredTheta() {}

    public ChassisSpeeds getSpeed() {
        return new ChassisSpeeds(0, 0, 0);
    }

    public void setOptimalRotationGetter(DoubleSupplier supplier) {}

    public void setupTheta(boolean isAuto) {};

    public Command alignToTowerY() {
        return CommandBuilder.none(this);
    }

    public CommandBuilder driveToPose(Supplier<Pose2d> target) {
        return CommandBuilder.none(this);
    }

    public CommandBuilder driveToPose(Supplier<Pose2d> target, List<Double> speedPercents) {
        return CommandBuilder.none(this);
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

    public Command temporarySlowmode() {
        return CommandBuilder.none(this);
    }
}