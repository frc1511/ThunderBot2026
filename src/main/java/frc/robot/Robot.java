// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsytem.Moving;

public class Robot extends TimedRobot {
    Moving moving;
    public Robot() {
        moving = new Moving();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        moving.switchLowTrueFalse.onFalse(moving.runForward());  
        moving.switchLowTrueFalse.onTrue(moving.stopMotor());    // Unfinished code i have to go - oliver  P.S DONT TOUCH IT!!! :3
        moving.switchHighTrueFalse.onFalse(moving.runBackward());
        moving.switchHighTrueFalse.onTrue(moving.stopMotor());
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
