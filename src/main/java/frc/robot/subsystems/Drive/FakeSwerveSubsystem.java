package frc.robot.subsystems.Drive;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.thunder.lib.trajectory.ThunderTrajectoryRunnerProperties;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.util.Constants.Status;

public class FakeSwerveSubsystem implements SwerveSubsystem {
    public FakeSwerveSubsystem() {
        
    }

    public void setFieldCentric(boolean isOn) {
    }

    public void setLimelightDisable(boolean isDisabled) {
    }

    public Command increaseSpeed() {
        return Commands.none();
    }
    
    public Command decreaseSpeed() {
        return Commands.none();
    }

    public Command setHubLock(Boolean isOn) {
        return Commands.none();
    }

    public Command driveWithJoysticks(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        return new InstantCommand(() -> {}, this);
    }

    public Command brick() {
        return Commands.none();
    }

    public Command pointWithController(DoubleSupplier leftX, DoubleSupplier leftY) {
        return Commands.none();
    }

    public Command idle() {
        return Commands.none();
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return Commands.none();
    }

    @Override
    public void periodic() {
        
    }
    
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        
    }

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
        return Commands.none();
    }

    public void resetControl(Pose2d pose) {
        
    }

    public ThunderTrajectoryRunnerProperties getTrajectoryRunnerProperties() {
        return null;
    }

    @Override
    public Status status() {
        return Status.DISABLED;
    }

    @Override
    public void seedFieldCentric() {
        
    }
}