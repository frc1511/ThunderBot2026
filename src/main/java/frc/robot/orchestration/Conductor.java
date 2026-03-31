package frc.robot.orchestration;

import java.util.List;

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

        new Trigger(this::inStartingConfiguration).whileTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.HOME));

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

        testingpose = Helpers.getTargetHangPose(m_robot.drivetrain.currentPose());
        testingfield.setRobotPose(testingpose);
        SmartDashboard.putData("testingpose", testingfield);
    }

    public Command autoHang() {
        HangSubsystem hang = m_robot.hang;
        SwerveSubsystem swerve = m_robot.drivetrain;
        return
            // Step 2/3: Lineup to the Y Position of the tower, Ensure that swerve is pointing in the correct direction
            swerve.alignToTowerY()
                  .alongWith(hang.extend())
            // Step 4: Lineup to the correct X position
            .andThen(
                swerve
                    .driveToPose(
                        () -> {
                            Pose2d currentPose = swerve.currentPose();

                            return Helpers.getTargetHangPose(currentPose);
                        },
                        List.of(.3d, .1d, .30d)) // List.of(.5d, .15d, .6d
                .withTimeout(4)
            )
            // Step 5: Retract Hang
            .andThen(
                hang.retract()
            );
    }
}
