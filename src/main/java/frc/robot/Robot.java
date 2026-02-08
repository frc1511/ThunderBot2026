// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsytem.Moving;
import frc.util.Alert;

public class Robot extends TimedRobot {
    Moving moving;
    DigitalInput m_SW_2;
    DigitalInput m_SW_3;

    public Robot() {
        moving = new Moving();
        m_SW_2 = new DigitalInput(2);
        m_SW_3 = new DigitalInput(3);

        // new Trigger(() -> m_SW_2.get()).onFalse(moving.runForward()).onTrue(moving.stop());  
        // new Trigger(() -> m_SW_3.get()).onFalse(moving.runBackward()).onTrue(moving.stop());

        new Trigger(() -> m_SW_2.get()).onTrue(new InstantCommand(() -> Alert.critical("DRIVE SAFETY", "SECONDARY SAFETY TRIPPED")).ignoringDisable(true));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
