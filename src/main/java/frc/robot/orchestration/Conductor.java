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

        new Trigger(m_robot.hungerOrchestrator::isIntaking).onTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.INTAKING));

        // new Trigger(blehhh :3).onTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.HUNG) // TODO: Hang

        new Trigger(this::cannonReady).onTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.FIRE_READY));

        new Trigger(this::inStartingConfiguration).onTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.HOME));

        new Trigger(this::trenchSafe).onTrue(m_robot.blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.TRENCH_SAFE));
    }

    public boolean cannonReady() {
        return m_robot.cannonOrchestrator.ready();
    }

    public boolean inStartingConfiguration() {
        return true;
    }

    public boolean trenchSafe() {
        return m_robot.cannonOrchestrator.hood.safeForTrench();
    }
}
