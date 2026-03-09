package frc.robot.orchestration;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Hang.HangSubsystem;
import frc.util.Constants;
import frc.util.Helpers;
import frc.util.ZoneConstants;

public class Conductor {
    public HubOrchestrator hub;
    
    private Robot m_robot;

    private Field2d testingfield;
    private Pose2d testingpose;

    public Conductor(Robot robot) {
        m_robot = robot;
        hub = robot.hubOrchestrator;

        new Trigger(m_robot.hungerOrchestrator::isIntaking).whileTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.INTAKING));

        new Trigger(m_robot.hang::climbClimbingButHasntClumbJustYet).whileTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.HUNG));

        new Trigger(this::cannonReady).whileTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.FIRE_READY));

        new Trigger(this::inStartingConfiguration).whileTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.HOME).ignoringDisable(true));

        new Trigger(this::trenchSafe).whileTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.TRENCH_SAFE));
    
        testingfield = new Field2d();
        testingpose = Pose2d.kZero;
    }

    public boolean cannonReady() {
        return m_robot.cannonOrchestrator.ready();
    }

    public boolean inStartingConfiguration() {  
        return m_robot.hood.safeForTrench() && m_robot.pivot.isIn();
    }

    public boolean trenchSafe() {
        return m_robot.cannonOrchestrator.hood.safeForTrench();
    }

    public void periodic() {
        if (m_robot.ledDisable.isOff()) m_robot.blinkyBlinkyOrchestrator.sparkle();

        // STEP 2
        // testingpose = new Pose2d(
        //     Helpers.isBlueAlliance() ? Constants.HangConstants.kTowerDistanceFromWallX : ZoneConstants.kFieldLength.magnitude() - Constants.HangConstants.kTowerDistanceFromWallX,
        //     m_robot.drivetrain.currentPose().getY(),
        //     // If left - 0, If right - 180
        //     ZoneConstants.isOnLeftSide(m_robot.drivetrain.currentPose().getTranslation()) ? Rotation2d.kZero : Rotation2d.k180deg
        // );

        // STEP 4
        // HangSubsystem hang = m_robot.hang;
        // SwerveSubsystem swerve = m_robot.drivetrain;
        // Pose2d currentPose = swerve.currentPose();
        // Pair<Double, Boolean> measurement = hang.getDistanceSensor();
        // m_distanceToTower = measurement.getFirst();
        // if (!measurement.getSecond()) {
        //     // Fudge the data rather than get random distance
        //     m_distanceToTower = .15;
        // } else {
        //     double distance = m_distanceToTower;
        //     if (distance <= .05) distance = 0;
        //     if (Helpers.isBlueAlliance()) distance *= -1;
        //     Pose2d target = new Pose2d(
        //         currentPose.getX(),
        //         currentPose.getY() + distance,
        //         currentPose.getRotation()
        //     );
        //     testingpose = target;
        // }

        testingfield.setRobotPose(testingpose);
        SmartDashboard.putData(testingfield);
    }

    private double m_distanceToTower = 0.0;
    public Command autoHang() {
        HangSubsystem hang = m_robot.hang;
        SwerveSubsystem swerve = m_robot.drivetrain;
        return
            // // Step 1: Extend Hang
            hang.extend()
            // // Step 2/3: Lineup to the Y Position of the tower, Ensure that swerve is pointing in the correct direction
            .andThen(
                swerve.alignToTowerY()
            )
            // // Step 4: Lineup to the correct X position using the hang sensor
            .andThen(
                swerve
                    .driveToPose(
                        () -> {
                            Pose2d currentPose = swerve.currentPose();
                            Pair<Double, Boolean> measurement = hang.getDistanceSensor();
                            m_distanceToTower = measurement.getFirst();
                            if (!measurement.getSecond()) {
                                // Alignment Fail
                                return swerve.currentPose();
                            }
                            if (m_distanceToTower <= .10) m_distanceToTower = 0;
                            if (Helpers.isBlueAlliance()) m_distanceToTower *= -1;
                            Pose2d target = new Pose2d(
                                currentPose.getX(),
                                currentPose.getY() + m_distanceToTower,
                                currentPose.getRotation()
                            );
                            return target;
                        })
                    .onInitialize(
                        () -> {
                            m_distanceToTower = hang.getDistanceSensor().getFirst();
                        }
                    )
            )
            // // Step 5: Retract Hang
            .andThen(
                hang.retract()
            );
    }
}
