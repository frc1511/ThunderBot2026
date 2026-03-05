package frc.robot.orchestration;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
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
}
