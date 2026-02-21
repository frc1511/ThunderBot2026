// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.thunder.lib.auto.ThunderAutoProject;
import com.thunder.lib.auto.ThunderAutoSendableChooser;
import com.thunder.lib.trajectory.ThunderTrajectoryRunnerProperties;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.orchestration.BlinkyBlinkyOrchestrator;
import frc.robot.orchestration.CannonOrchestrator;
import frc.robot.orchestration.Conductor;
import frc.robot.orchestration.FiringOrchestrator;
import frc.robot.orchestration.HubOrchestrator;
import frc.robot.orchestration.HungerOrchestrator;
import frc.robot.orchestration.Autonomous.AutoLoader;
import frc.robot.subsystems.Cannon.HoodSubsystem;
import frc.robot.subsystems.Cannon.ShooterSubsystem;
import frc.robot.subsystems.Cannon.TurretSubsystem;
import frc.robot.subsystems.Drive.FakeSwerveSubsystem;
import frc.robot.subsystems.Drive.RealSwerveSubsystem;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Hang.HangSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.PivotSubsystem;
import frc.robot.subsystems.Storage.KickerSubsystem;
import frc.robot.subsystems.Storage.SpindexerSubsystem;
import frc.util.Alert;
import frc.util.Broken;
import frc.util.Constants;
import frc.util.ThunderSwitchboard;
import frc.util.ThunderSwitchboard.ThunderSwitch;

public class Robot extends TimedRobot {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController auxController = new CommandXboxController(1);
    private final ThunderSwitchboard switchBoard = new ThunderSwitchboard(2);

    // private final Telemetry logger = new Telemetry(Constants.SwerveConstants.kMaxSpeed);
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();
  
    public final SwerveSubsystem drivetrain = Broken.drivetrainFullyDisabled ? new FakeSwerveSubsystem() : new RealSwerveSubsystem();

    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final HoodSubsystem hood = new HoodSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();

    public final SpindexerSubsystem spindexer = new SpindexerSubsystem();
    public final KickerSubsystem kicker = new KickerSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final PivotSubsystem pivot = new PivotSubsystem();   
    public final HangSubsystem hang = new HangSubsystem();

    public final SafetyWatchdog safetyWatchdog;

    public final BlinkyBlinkyOrchestrator blinkyBlinkyOrchestrator;
    public final CannonOrchestrator cannonOrchestrator;
    public final FiringOrchestrator firingOrchestrator;
    public final HubOrchestrator hubOrchestrator;
    public final HungerOrchestrator hungerOrchestrator;

    public final Conductor conductor;

    private ThunderAutoSendableChooser autoChooser;

    public ThunderSwitch trevorDisable = switchBoard.button(1);
    public ThunderSwitch emmaDisable = switchBoard.button(2);
    public ThunderSwitch auxManual = switchBoard.button(3);
    public ThunderSwitch fieldCentric = switchBoard.button(4);
    public ThunderSwitch ledDisable = switchBoard.button(5);
    public ThunderSwitch limelightDisable = switchBoard.button(6);
    public ThunderSwitch climberDisable = switchBoard.button(7);
    public ThunderSwitch placeHolder8 = switchBoard.button(8); // Yes, the placeholder switches have different slots than their names. This is because drive team is 1-based rather than 0-based and this aligns with the controller map.
    public ThunderSwitch placeHolder9 = switchBoard.button(9);
    public ThunderSwitch placeHolder10 = switchBoard.button(10);
    public ThunderSwitch pitMode = switchBoard.button(11);

    public Robot() {
        // DataLogManager.start(); //* Uncomment for logs
        Alert.info("The robot has restarted");

        // MARK: Drive

        if (!Broken.drivetrainFullyDisabled) {
            drivetrain.setDefaultCommand(
                drivetrain
                    .driveWithJoysticks(driverController::getLeftX, driverController::getLeftY, driverController::getRightX)
                    .onlyIf(() -> !driverController.y().getAsBoolean())
                    .onlyIf(trevorDisable::isOff)
                    .withName("DriveWithJoysticks")
            );
        }

        RobotModeTriggers.disabled().whileTrue(drivetrain.idle());

        driverController.a().and(trevorDisable::isOff).whileTrue(drivetrain.brick());
        driverController.b().and(trevorDisable::isOff).whileTrue(drivetrain.pointWithController(driverController::getLeftX, driverController::getLeftY));

        // Reset the field-centric heading on left bumper press. 
        driverController.x().and(trevorDisable::isOff).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        driverController.leftBumper().and(trevorDisable::isOff).onTrue(drivetrain.decreaseSpeed());
        driverController.rightBumper().and(trevorDisable::isOff).onTrue(drivetrain.increaseSpeed());

        driverController.y().and(trevorDisable::isOff).whileTrue(drivetrain.driveLockedToArcWithJoysticks(driverController::getLeftX));
        new Trigger(() -> 
            Math.abs(driverController.getRightTriggerAxis()) > Constants.kControllerDeadzone)
            .and(trevorDisable::isOff)
            .onTrue(drivetrain.setHubLock(true))
            .onFalse(drivetrain.setHubLock(false));

        // SysID
        // driverController.start().and(driverController.y()).and(trevorDisable::isOff).whileTrue(drivetrain.sysID.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).and(trevorDisable::isOff).whileTrue(drivetrain.sysID.sysIdQuasistatic(Direction.kReverse));
        // driverController.back().and(driverController.y()).and(trevorDisable::isOff).whileTrue(drivetrain.sysID.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).and(trevorDisable::isOff).whileTrue(drivetrain.sysID.sysIdDynamic(Direction.kReverse));
        // drivetrain.registerTelemetry(logger::telemeterize);

        // MARK: Aux
        // hang.setDefaultCommand(hang.halt());
        // auxController.y().and(emmaDisable::isOff)
        //     .whileTrue(
        //         hang.zeroHang()
        //         .onlyIf(auxManual::isOff)
        //         .onlyIf(climberDisable::isOff)
        //     )
        //     .onFalse(hang.halt());
        // auxController.a().and(emmaDisable::isOff)
        //     .whileTrue(
        //         hang.retract()
        //         .onlyIf(auxManual::isOff)
        //         .onlyIf(climberDisable::isOff)
        //     )
        //     .onFalse(hang.halt());
        // auxController.b().and(emmaDisable::isOff)
        //     .whileTrue(
        //         hang.extend()
        //         .onlyIf(auxManual::isOff)
        //         .onlyIf(climberDisable::isOff)
        //     )
        //     .onFalse(hang.halt());

        
        
        
        hood.setDefaultCommand(hood.halt());
        shooter.setDefaultCommand(shooter.halt());
        kicker.setDefaultCommand(kicker.halt());

        auxController.b().whileTrue(shooter.runAtCustomSpeed(() -> 4000d));

        auxController.x()
            .whileTrue(shooter.runAtCustomSpeed(() -> SmartDashboard.getNumber("num", 2100))
                .alongWith(
                    kicker.run()
                        .alongWith(spindexer.spin(Constants.Storage.Spindexer.Duration.INTAKE.get()))
                        .onlyIf(() -> shooter.shooterAtSpeed())
                )
            );
        auxController.y()
            .onTrue(hood.toPosition(() -> 0.5d));
        auxController.a()
            .onTrue(intake.eat()).onFalse(intake.stopEating());
                    
        auxController.leftBumper().onTrue(pivot.manual_pivot(() -> auxController.getLeftY() * -0.2));
        auxController.back()
                    .whileTrue(pivot.pivotUp()).onFalse(pivot.halt());                        
        auxController.start()
                    .whileTrue(pivot.pivotDown()).onFalse(pivot.halt());


        // auxController.a().and(emmaDisable::isOff).onTrue(kicker.playSoccer());
        // auxController.a().and(emmaDisable::isOff).onFalse(kicker.halt());

        // MARK: Orchestration

        blinkyBlinkyOrchestrator = new BlinkyBlinkyOrchestrator(this);
        cannonOrchestrator = new CannonOrchestrator(this);
        firingOrchestrator = new FiringOrchestrator(this);
        hubOrchestrator = new HubOrchestrator(this);
        hungerOrchestrator = new HungerOrchestrator(this);

        conductor = new Conductor(this);

        // MARK: Auto

        ThunderAutoProject autoProject = AutoLoader.load(this);

        autoChooser = new ThunderAutoSendableChooser("Auto_Mode");

        autoChooser.includeProjectSource(autoProject);
        autoChooser.addAllAutoModesFromProject(autoProject.getName());
        autoChooser.addTrajectoryFromProject(autoProject.getName(), "ShootFromStart");
        autoChooser.addTrajectoryFromProject(autoProject.getName(), "teehee");
        ThunderTrajectoryRunnerProperties props = drivetrain.getTrajectoryRunnerProperties();
        if (props != null) {
            autoChooser.setTrajectoryRunnerProperties(drivetrain.getTrajectoryRunnerProperties());
        }

        safetyWatchdog = new SafetyWatchdog(this);
    }

    @SuppressWarnings("all") // Identical Expressions Warning Suppression (BuildConsts)
    @Override
    public void robotInit() {
        Alert.info(String.format("Last build time: %s, on branch %s.", BuildConstants.BUILD_DATE, BuildConstants.GIT_BRANCH) + (BuildConstants.DIRTY == 1 ? "Modified" : ""));
    }

    private int i = 0; // For the Frozen Dashboard Detector 2000

    @Override
    public void robotPeriodic() {
        if (RobotController.getBatteryVoltage() < 9) {
            // Alert.critical(String.format("Battery voltage dipped below 9v, reached %.2f", RobotController.getBatteryVoltage()));
        } else if (RobotController.getBatteryVoltage() < 10) {
            Alert.error("Battery voltage dipped below 10v");
        }

        if (!driverController.isConnected()) {
            Alert.error("Drive Controller Disconnected");
        }

        if (!auxController.isConnected()) {
            Alert.error("Aux Controller Disconnected");
        }

        SmartDashboard.putNumber("Frozen_Dashboard_Detector_2000", i++);
        SmartDashboard.putNumber("BATTERY_voltage", RobotController.getBatteryVoltage());


        m_timeAndJoystickReplay.update();
        SmartDashboard.putData(CommandScheduler.getInstance());
        
        drivetrain.setFieldCentric(fieldCentric.isOn());
        drivetrain.setLimelightDisable(limelightDisable.isOn());
        
        conductor.periodic();
        blinkyBlinkyOrchestrator.sparkle();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        Command autoCommand = autoChooser.getSelectedCommand();
        if (autoCommand != Commands.none()) {
            CommandScheduler.getInstance().schedule(autoCommand);
        }
    }

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
