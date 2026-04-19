// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.orchestration.BlinkyBlinkyOrchestrator;
import frc.util.Alert;
import frc.util.Broken;
import frc.util.Helpers;
import frc.util.Thunder.ThunderSwitchboard;
import frc.util.Thunder.Tuneable;
import frc.util.Thunder.ThunderSwitchboard.ThunderSwitch;

public class Robot extends TimedRobot {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController auxController = new CommandXboxController(1);
    private final ThunderSwitchboard switchBoard = new ThunderSwitchboard(2);

    public final BlinkyBlinkyOrchestrator blinkyBlinkyOrchestrator;

    public ThunderSwitch driveDisable = switchBoard.button(1);
    public ThunderSwitch auxDisable = switchBoard.button(2);
    public ThunderSwitch pitModePlatinumEditionTM = switchBoard.button(3);
    public ThunderSwitch fieldCentric = switchBoard.button(4);
    public ThunderSwitch ledDisable = switchBoard.button(5);
    public ThunderSwitch limelightDisable = switchBoard.button(6);
    public ThunderSwitch climberDisable = switchBoard.button(7);
    /** DO NOT USE THIS, instead use pitModePlus */
    public ThunderSwitch _DNU_pitModePlusPhysical = switchBoard.button(8);
    public ThunderSwitch bypassMode = switchBoard.button(9);
    public ThunderSwitch oneDriverMode = switchBoard.button(10);
    public ThunderSwitch pitMode = switchBoard.button(11);

    public ThunderSwitch pitModePlus = switchBoard.new ThunderSwitch(new Trigger(() -> _DNU_pitModePlusPhysical.isOn() && pitMode.isOn()));

    public PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);

    public Robot() {
        blinkyBlinkyOrchestrator = new BlinkyBlinkyOrchestrator(this);
        blinkyBlinkyOrchestrator.bootStatus(0);
        blinkyBlinkyOrchestrator.bootStatus(1);

        Alert.info("The robot has restarted");
        DriverStation.silenceJoystickConnectionWarning(true); // This hopefully helps with radio issues

        blinkyBlinkyOrchestrator.bootStatus(2);

        { // MARK: Switchboard
            ledDisable.get()
                .whileTrue(
                    new InstantCommand(() -> {
                        Broken.blinkyBlinkyButtonBopped = true;
                    })
                    .ignoringDisable(true).repeatedly()
                )
                .onFalse(
                    new InstantCommand(() -> {
                        Broken.blinkyBlinkyButtonBopped = false;
                    }).ignoringDisable(true)
                );

            Helpers.setPitModePlus(pitModePlus.isOn());
            pitModePlus.get().onChange(new InstantCommand(() -> Helpers.setPitModePlus(pitModePlus.isOn())).ignoringDisable(true));

            Helpers.setPitMode(pitMode.isOn());
            pitMode.get().onChange(new InstantCommand(() -> Helpers.setPitMode(pitMode.isOn())).ignoringDisable(true));

            Helpers.setBypassMode(bypassMode.isOn());
            bypassMode.get().onChange(new InstantCommand(() -> Helpers.setBypassMode(bypassMode.isOn())).ignoringDisable(true));
        }

        blinkyBlinkyOrchestrator.bootStatus(3);

        SmartDashboard.putData(CommandScheduler.getInstance());

        SmartDashboard.putData("Extra / SwitchBoard", switchBoard);

        SmartDashboard.putData("Code Broken Disables", Broken.getBrokenStatuses());

        blinkyBlinkyOrchestrator.bootStatus(4);

        blinkyBlinkyOrchestrator.startRound();
    }    

    @SuppressWarnings("all") // Identical Expressions Warning Suppression (BuildConsts)
    @Override
    public void robotInit() {
        Alert.info(String.format("Last build time: %s, on branch %s.", BuildConstants.BUILD_DATE, BuildConstants.GIT_BRANCH) + (BuildConstants.DIRTY == 1 ? "Modified" : ""));
    }

    private int i = 0; // For the Frozen Dashboard Detector 2000

    @Override
    public void robotPeriodic() {
        Tuneable.periodic();

        blinkyBlinkyOrchestrator.batteryVoltage = PDH.getVoltage();

        if (!driverController.isConnected()) {
            Alert.error("Drive Controller Disconnected");
        }

        if (!auxController.isConnected()) {
            Alert.error("Aux Controller Disconnected");
        }

        SmartDashboard.putBoolean("Extra / Drive Disabled", driveDisable.isOn());
        SmartDashboard.putBoolean("Extra / Aux Disabled", auxDisable.isOn());

        SmartDashboard.putNumber("Extra / Frozen Dashboard Detector 2000", i++);

        SmartDashboard.putBoolean("Extra / Driver Aid / Hub Active", Helpers.isHubActive(true));
        SmartDashboard.putNumber("Extra / Driver Aid / Match Timer", Timer.getMatchTime());

        SmartDashboard.putBoolean("Extra / Bypass Mode", Helpers.isBypassModeEnabled());
        SmartDashboard.putBoolean("Extra / Pit Mode+ Mode", Helpers.isPitModePlusEnabled());
        SmartDashboard.putBoolean("Extra / Pit Mode Platinum Mode", pitModePlatinumEditionTM.isOn());

        if (driverController.a().getAsBoolean()) {
            blinkyBlinkyOrchestrator.startRound();
        }
        if (driverController.b().getAsBoolean()) {
            blinkyBlinkyOrchestrator.restart();
        }
        blinkyBlinkyOrchestrator.sparkle(driverController.getLeftY() > .7, driverController.getLeftY() < -.7, auxController.getLeftY() > .7, auxController.getLeftY() < -.7);
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

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
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
