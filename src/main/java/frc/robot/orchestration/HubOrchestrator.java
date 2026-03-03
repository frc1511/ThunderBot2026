package frc.robot.orchestration;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        runConvergance();
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

    private Pair<FiringDataPoint, Double> converge(ChassisSpeeds currentSpeed, Translation2d robotPosition, Translation2d targetPosition) {
        ArrayList<Translation2d> iterations = new ArrayList<Translation2d>();

        Translation2d initialDeltaTranslation = targetPosition.minus(robotPosition);

        Translation2d currentDeltaPosition = initialDeltaTranslation;
        Double previousTimeOfFlight = null;

        for (int iteration = 0; iteration < Constants.Swerve.kTimeOfFlightConvergenceMaxRecursions; iteration++) {
            double previousTau;
            if (previousTimeOfFlight != null) {
                previousTau = previousTimeOfFlight.doubleValue();
            } else {
                double deltaDistance = currentDeltaPosition.getNorm();
                previousTau = firingTable.lerp(deltaDistance).timeOfFlight;
            }

            Translation2d virtualTargetOffset = new Translation2d(
                previousTau * -currentSpeed.vxMetersPerSecond,
                previousTau * -currentSpeed.vyMetersPerSecond
            );

            Translation2d virtualTargetPosition = targetPosition.plus(virtualTargetOffset);

            Translation2d virtualTargetDelta = virtualTargetPosition.minus(robotPosition);
            double deltaDistance = virtualTargetDelta.getNorm();
            double newTau = firingTable.lerp(deltaDistance).timeOfFlight;

            Translation2d realTrajectoryEnd = new Translation2d(
                virtualTargetPosition.getX() + currentSpeed.vxMetersPerSecond * newTau,
                virtualTargetPosition.getY() + currentSpeed.vyMetersPerSecond * newTau
            );

            iterations.add(realTrajectoryEnd);

            if (previousTimeOfFlight != null && Math.abs(newTau - previousTau) < Constants.Swerve.kTimeOfFlightConvergenceTolerance) {
                break;
            }
            
            currentDeltaPosition = virtualTargetPosition.minus(robotPosition);

            previousTimeOfFlight = newTau;
        }

        Translation2d finalTargetPosition = iterations.get(iterations.size() - 1);

        Translation2d deltaTarget = finalTargetPosition.minus(robotPosition);
        double finalDistance = deltaTarget.getNorm();
        double finalTheta = deltaTarget.getAngle().getRadians();

        FiringDataPoint finalPoint = firingTable.lerp(finalDistance);
        
        return new Pair<FiringTable.FiringDataPoint,Double>(finalPoint, finalTheta);
    }

    public void runConvergance() {
        ChassisSpeeds currentSpeed = swerveSubsystem.getSpeed();
        Pose2d currentPose = swerveSubsystem.currentPose();
        Pose2d nearestHub = Helpers.allianceHub();

        SmartDashboard.putNumber("converge_dist", Math.sqrt(Math.pow(currentPose.getX() - nearestHub.getX(), 2) + Math.pow(currentPose.getY() - nearestHub.getY(), 2)));
        
        latestConvergance = converge(currentSpeed, currentPose.getTranslation(), nearestHub.getTranslation());
        SmartDashboard.putNumber("firingPoint_speed", latestConvergance.getFirst().speedRPM);
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
