package frc.robot.orchestration;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Hang.HangSubsystem;
import frc.util.Constants;

public class Conductor {
    public HubOrchestrator hub;
    
    private Robot m_robot;

    public Conductor(Robot robot) {
        m_robot = robot;
        hub = robot.hubOrchestrator;

        new Trigger(m_robot.hungerOrchestrator::isIntaking).whileTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.INTAKING));

        new Trigger(m_robot.hang::climbClimbingButHasntClumbJustYet).whileTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.HUNG));

        new Trigger(this::cannonReady).whileTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.FIRE_READY));

        new Trigger(this::inStartingConfiguration).whileTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.HOME).ignoringDisable(true));

        new Trigger(this::trenchSafe).whileTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.TRENCH_SAFE));
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
    }

    private double m_distanceToTower = 0.0;
    public Command autoHang() {
        HangSubsystem hang = m_robot.hang;
        SwerveSubsystem swerve = m_robot.drivetrain;
        return
            // Step 1: Extend Hang
            hang.extend()
            // Step 2/3: Lineup to the Y Position of the tower, Ensure that swerve is pointing in the correct direction
            .andThen(
                swerve.alignToTowerY()
            )
            // Step 4: Lineup to the correct X position using the hang sensor
            .andThen(
                swerve
                    .driveToPose(
                        () -> {
                            Pose2d currentPose = swerve.currentPose();
                            Pair<Double, Boolean> measurement = hang.getDistanceSensor();
                            if (!measurement.getSecond()) {
                                // DO NOT TRUST DA MEASUREMENT
                            }
                            m_distanceToTower = measurement.getFirst();
                            double distance = m_distanceToTower;
                            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) distance *= -1;
                            return new Pose2d(
                                currentPose.getX() + distance,
                                currentPose.getY(),
                                currentPose.getRotation()
                            );
                        })
                    .onInitialize(
                        () -> {
                            m_distanceToTower = hang.getDistanceSensor().getFirst();
                        })
            )
            // Step 5: Retract Hang
            .andThen(
                hang.retract()
            );
    }
}
