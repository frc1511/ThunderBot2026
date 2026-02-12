package frc.robot.orchestration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.orchestration.CannonOrchestrator.Orientation;
import frc.robot.subsystems.Drive.SwerveSubsystem;

public class HubOrchestrator {
    CannonOrchestrator cannonOrchestrator;
    SwerveSubsystem swerveSubsystem;
    
    public HubOrchestrator(Robot robot) {
        cannonOrchestrator = robot.cannonOrchestrator;
        swerveSubsystem = robot.drivetrain;
    }

    public double hubLockTurretAngle() {
        Pose2d nearestHub = new Pose2d(
            11.887319,
            7.41196,
            Rotation2d.kZero
        );
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
}
