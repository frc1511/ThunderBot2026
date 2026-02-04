// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsytems.HangSubsystem;

public class Robot extends TimedRobot {

    private CommandXboxController m_auxController = new CommandXboxController(1);

    public HangSubsystem hang = new HangSubsystem();

    public Robot() {
        hang.setDefaultCommand(hang.halt());
        m_auxController.y().onTrue(hang.zeroHang());
        m_auxController.a().onTrue(hang.retract());
        m_auxController.b().onTrue(hang.extend());
    }

    private int i = 0;

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Frozen_Dashboard_Detector_2000", i++);
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
