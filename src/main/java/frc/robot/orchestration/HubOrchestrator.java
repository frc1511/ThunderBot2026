package frc.robot.orchestration;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import frc.robot.orchestration.CannonOrchestrator.Orientation;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.util.Constants;
import frc.util.FiringTable;
import frc.util.Helpers;
import frc.util.FiringTable.FiringDataPoint;

public class HubOrchestrator {
    CannonOrchestrator cannonOrchestrator;
    SwerveSubsystem swerveSubsystem;
    FiringTable firingTable;
    
    Pair<FiringDataPoint, Double> latestConvergance;

    public HubOrchestrator(Robot robot) {
        cannonOrchestrator = robot.cannonOrchestrator;
        swerveSubsystem = robot.drivetrain;
        firingTable = new FiringTable();
    }

    public double hubLockTurretAngle() {
        Pose2d nearestHub = Helpers.allianceHub();

        Pose2d currentPose = swerveSubsystem.currentPose();

        double dX = currentPose.getX() - nearestHub.getX();
        double dY = currentPose.getY() - nearestHub.getY();

        return Math.atan2(dY, dX) - currentPose.getRotation().getRadians();
    }

    public double hubLockHoodAngle() {
        return 0;
    }

    public Command turretAutoLock() {
        return cannonOrchestrator.moveToOrientation(new Orientation(hubLockTurretAngle(), hubLockHoodAngle()));
    }

    private Pair<FiringDataPoint, Double> converge(ChassisSpeeds currentSpeed, Translation2d robotPosition, Translation2d targetPosition, int recursions) {
        // These are relative velocities of the hub to the robot. The hub is not moving, but from the robot's perspective it is moving with the opposite velocity of the robot.
        double relativeVelocityOfHubX = -currentSpeed.vxMetersPerSecond;
        double relativeVelocityOfHubY = -currentSpeed.vyMetersPerSecond;

        double dX = robotPosition.getX() - targetPosition.getX();
        double dY = robotPosition.getY() - targetPosition.getY();

        FiringDataPoint interpolatedDataPoint = firingTable.lerp(targetPosition.getDistance(robotPosition));

        double timeOfFlight = interpolatedDataPoint.timeOfFlight;
    
        double dXPredicted = dX + relativeVelocityOfHubX * timeOfFlight;
        double dYPredicted = dY + relativeVelocityOfHubY * timeOfFlight;
    
        double distPredicted = Math.sqrt(Math.pow(dYPredicted, 2) + Math.pow(dXPredicted, 2));
    
        FiringDataPoint nextInterpolatedDataPoint = firingTable.lerp(distPredicted);

        double nextTimeOfFlight = nextInterpolatedDataPoint.timeOfFlight;

        double timeOfFlightChange = nextTimeOfFlight - timeOfFlight;

        if (Math.abs(timeOfFlightChange) > Constants.Swerve.kTimeOfFlightConvergenceTolerance && !(recursions >= Constants.Swerve.kTimeOfFlightConvergenceMaxRecursions)) {
            Translation2d newTargetPosition = new Translation2d(dXPredicted, dYPredicted);
            return converge(currentSpeed, robotPosition, newTargetPosition, recursions + 1);
        } else {
            return new Pair<FiringDataPoint, Double>(nextInterpolatedDataPoint, Math.atan(dYPredicted / dXPredicted));
        }
    }

    public void runConvergance() {
        ChassisSpeeds currentSpeed = swerveSubsystem.getSpeed();
        Pose2d currentPose = swerveSubsystem.currentPose();
        Pose2d nearestHub = Helpers.allianceHub();

        latestConvergance = converge(currentSpeed, currentPose.getTranslation(), nearestHub.getTranslation(), 0);
    }

    
    public double getOptimalShootSpeed() {
        return latestConvergance.getFirst().speedRPM;
    }

    public double getOptimalHoodAngle() {
        return latestConvergance.getFirst().hoodAngle;
    }

    public double getOptimalDriveOrientation() {
        return latestConvergance.getSecond();
    }
}
