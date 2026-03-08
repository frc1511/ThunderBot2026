package frc.robot.subsystems.Drive;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.thunder.lib.trajectory.ThunderTrajectoryRunnerProperties;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import frc.util.CommandBuilder;
import frc.util.Constants.Status;
import frc.util.Thunder.ThunderInterface;

public interface SwerveSubsystem extends ThunderInterface {

    public void setSpeedMultiplier(double speed);

    public void setFieldCentric(boolean isOn);

    public void setLimelightDisable(boolean isDisabled);

    public Command increaseSpeed();
    
    public Command decreaseSpeed();

    public Command setHubLock(Boolean isOn);

    public Command hubLock();

    public Command trenchLock();

    public Command driveWithJoysticks(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX);

    public Command brick();

    public Command pointWithController(DoubleSupplier leftX, DoubleSupplier leftY);

    public Command idle();

    public Command applyRequest(Supplier<SwerveRequest> request);

    public Command toggleFieldCentric();

    @Override
    public void periodic();
    
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds);

    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    );

    public Optional<Pose2d> samplePoseAt(double timestampSeconds);

    public Pose2d currentPose();

    public void setupTheta(boolean isAuto);

    public boolean isCANSafe();

    public Command driveLockedToArcWithJoysticks(DoubleSupplier leftX);

    public void resetControl(Pose2d pose);

    public ThunderTrajectoryRunnerProperties getTrajectoryRunnerProperties();

    public void seedFieldCentric();

    @Override
    public Status status();

    public void ensureTheta(DoubleSupplier thetaSupplier);

    public void clearEnsuredTheta();

    public ChassisSpeeds getSpeed();

    public void setOptimalRotationGetter(DoubleSupplier supplier);

    public Command alignToTowerY();

    public CommandBuilder driveToPose(Supplier<Pose2d> target);
}
