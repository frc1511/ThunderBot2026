// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import com.thunder.lib.auto.ThunderAutoProject;
import com.thunder.lib.auto.ThunderAutoSendableChooser;
import com.thunder.lib.trajectory.ThunderTrajectoryRunnerProperties;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.ZoneConstants;
import frc.util.Thunder.ThunderSwitchboard;
import frc.util.Thunder.ThunderSwitchboard.ThunderSwitch;

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

    public ThunderSwitch driveDisable = switchBoard.button(1);
    public ThunderSwitch auxDisable = switchBoard.button(2);
    public ThunderSwitch placeholder3 = switchBoard.button(3);
    public ThunderSwitch fieldCentric = switchBoard.button(4);
    public ThunderSwitch ledDisable = switchBoard.button(5);
    public ThunderSwitch limelightDisable = switchBoard.button(6);
    public ThunderSwitch climberDisable = switchBoard.button(7);
    public ThunderSwitch placeHolder8 = switchBoard.button(8); // Yes, the placeholder switches have different slots than their names. This is because drive team is 1-based rather than 0-based and this aligns with the controller map.
    public ThunderSwitch placeHolder9 = switchBoard.button(9);
    public ThunderSwitch oneDriverMode = switchBoard.button(10);
    public ThunderSwitch pitMode = switchBoard.button(11);

    public Robot() {
        CommandScheduler.getInstance().registerSubsystem(pivot);
        CommandScheduler.getInstance().registerSubsystem(intake);
        // DataLogManager.start(); //* Uncomment for logs
        
        ledDisable.get().onTrue(new InstantCommand(() -> Broken.blinkyBlinkyButtonBopped = true)).onFalse(new InstantCommand(() -> Broken.blinkyBlinkyButtonBopped = false));
    
        // MARK: Orchestration

        blinkyBlinkyOrchestrator = new BlinkyBlinkyOrchestrator(this);
        cannonOrchestrator = new CannonOrchestrator(this);
        firingOrchestrator = new FiringOrchestrator(this);
        hubOrchestrator = new HubOrchestrator(this);
        hungerOrchestrator = new HungerOrchestrator(this);

        conductor = new Conductor(this);

        safetyWatchdog = new SafetyWatchdog(this);

        shooter.setOptimalSpeedGetter(hubOrchestrator::getOptimalShootSpeed);
        hood.setOptimalAngleGetter(hubOrchestrator::getOptimalHoodAngle);

        Alert.info("The robot has restarted");
        DriverStation.silenceJoystickConnectionWarning(true); // trying to fix radio problem
        SignalLogger.enableAutoLogging(false);

        // MARK: Drive
        if (!Broken.drivetrainFullyDisabled) { // driving with joysticks
            drivetrain.setDefaultCommand(
                drivetrain
                    .driveWithJoysticks(driverController::getLeftX, driverController::getLeftY, driverController::getRightX)
                    .onlyIf(() -> !driverController.y().getAsBoolean() || driveDisable.isOff())
                    .withName("DriveWithJoysticks")
            );
        };

        RobotModeTriggers.disabled().onTrue(drivetrain.idle().withTimeout(.1));

        driverController.x().and(driveDisable::isOff).whileTrue(drivetrain.brick()); // polymorphs the robot into a brick (hold) upon release polymorphs the brick back into a robot
        driverController.y().and(driveDisable::isOff).whileTrue(drivetrain.toggleHubLock()); // lock and shoot
        // driverController.b() // TODO: spooky climber shake, goal is to have the hang fully extended and wiggle to make it onto the tower
        driverController.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)); // reset IMU

        driverController.povUp().and(driveDisable::isOff).whileTrue(hang.extend()).onFalse(hang.halt()); // hang go uppies (hold)
        driverController.povDown().and(driveDisable::isOff).whileTrue(hang.retract()).onFalse(hang.halt()); // hang go downies (hold)
        driverController.povLeft().and(driveDisable::isOff).onTrue(drivetrain.increaseSpeed()); // drive go weeee
        driverController.povRight().and(driveDisable::isOff).onTrue(drivetrain.decreaseSpeed()); // drive go snail

        // driverController.leftTrigger() // trench align
        //     .whileTrue(
        //         TODO: trench align
        //     );
        driverController.rightTrigger() // temporary robot centric
            .whileTrue(
                new CommandBuilder()
                    .onExecute(() -> drivetrain.setFieldCentric(fieldCentric.isOn()))
                    .isFinished(true)
                    .onlyIf(() -> driveDisable.isOff() && oneDriverMode.isOff())
            )
            .onFalse(
                new CommandBuilder()
                    .onExecute(() -> drivetrain.setFieldCentric(!fieldCentric.isOn()))
                    .isFinished(true)
            );

        // Puts the hang and hood in down mode when going under trench
        new Trigger(
            () -> {
                ZoneConstants.ZoneInfo zone = ZoneConstants.checkZone(drivetrain.currentPose().getTranslation());
                return zone.isWithinOne && !zone.isBump;
        }).whileTrue(
            hang.stowForTrench()
                .alongWith(
                    hood.stowForTrench()
                )
        );
        // MARK: One Driver Mode
        driverController.rightTrigger() // outtake (hold)
            .whileTrue(
                intake.outtake()
                .onlyIf(() -> driveDisable.isOff() && oneDriverMode.isOn())
            );
        driverController.rightBumper() // preheat (hold)
            .whileTrue(
                shooter.preheat()
                .onlyIf(() -> driveDisable.isOff() && oneDriverMode.isOn())
            );
        driverController.rightTrigger() // intake (hold)
            .whileTrue(
                hungerOrchestrator.consume()
                .onlyIf(() -> driveDisable.isOff() && oneDriverMode.isOn()).onlyIf(driveDisable::isOff)
            );
        driverController.rightBumper() // fire (hold)
            .whileTrue(
                kicker.run().alongWith(
                    shooter.holdSpeedForShoot().alongWith(
                        spindexer.spin(Constants.Storage.Spindexer.Duration.FOREVER)
                    )
                )
                .onlyIf(() -> driveDisable.isOff() && oneDriverMode.isOn())
            );
        // MARK: Aux
        auxController.y() // Feed
            .whileTrue(
                spindexer.spin(Constants.Storage.Spindexer.Duration.FOREVER)
            .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
            )
            .onFalse(spindexer.halt());

        auxController.x() // Tower
            .onTrue(
                hood.toPosition(() -> Constants.Hood.Position.TRENCH.get()) // TODO: evil trench please change this
            .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
            );

        auxController.b() // Trench
            .onTrue(
                hood.toPosition(() -> Constants.Hood.Position.TRENCH.get())
            .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
            );

        auxController.a() // HUB
            .onTrue(
                hood.toPosition(() -> Constants.Hood.Position.HUB.get())
            .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
            );

        auxController.y() // feed (hold)
            .whileTrue(
                hood.toPosition(() -> Constants.Hood.Position.TOP.get())
                .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
            )
            .onFalse(hood.toPosition(() -> Constants.Hood.Position.BOTTOM.get()));

        auxController.leftBumper() // Preheat (hold)
            .whileTrue(
                shooter.preheat()
            .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
            )
            .onFalse(shooter.halt());

        auxController.leftTrigger() // Fire (hold)
            .whileTrue(
                firingOrchestrator.fire()
            .onlyIf(auxDisable::isOff)
            .onlyIf(oneDriverMode::isOff)
            )
            .onFalse(kicker.halt());

        auxController.rightBumper() // Outtake (hold)
            .whileTrue(
                intake.outtake() 
            .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
            )
            .onFalse(intake.stopEating());

        auxController.rightTrigger() // Intake (hold)
            .whileTrue(
                hungerOrchestrator.consume()
            .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
            )
            .onFalse(intake.stopEating());
        
        
        auxController.back() // Intake (hold)
            .whileTrue(
                pivot.pivotUp()
            .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
            );

        auxController.start() // Hood down
            .onTrue(
                hood.toPosition(() -> Constants.Hood.Position.BOTTOM.get())
            .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
            );
        
        hood.setDefaultCommand(hood.halt());
        shooter.setDefaultCommand(shooter.halt());
        kicker.setDefaultCommand(kicker.halt());

        // MARK: Auto
        ThunderAutoProject autoProject = AutoLoader.load(this);

        autoChooser = new ThunderAutoSendableChooser("Auto_Mode");

        autoChooser.includeProjectSource(autoProject);
        autoChooser.addAllAutoModesFromProject(autoProject.getName());
        autoChooser.addTrajectoryFromProject(autoProject.getName(), "ShootFromStart");
        autoChooser.addTrajectoryFromProject(autoProject.getName(), "test_teehee");
        autoChooser.addTrajectoryFromProject(autoProject.getName(), "test_complex_shoot");
        ThunderTrajectoryRunnerProperties props = drivetrain.getTrajectoryRunnerProperties();
        if (props != null) {
            autoChooser.setTrajectoryRunnerProperties(drivetrain.getTrajectoryRunnerProperties());
        }
        
        SmartDashboard.putData(CommandScheduler.getInstance());
    }    

    @SuppressWarnings("all") // Identical Expressions Warning Suppression (BuildConsts)
    @Override
    public void robotInit() {
        Alert.info(String.format("Last build time: %s, on branch %s.", BuildConstants.BUILD_DATE, BuildConstants.GIT_BRANCH) + (BuildConstants.DIRTY == 1 ? "Modified" : ""));
    }
    
    private int i = 0; // For the Frozen Dashboard Detector 2000
    
    @Override
    public void robotPeriodic() {
        if (!driverController.isConnected()) {
            Alert.error("Drive Controller Disconnected");
        }
        
        if (!auxController.isConnected()) {
            Alert.error("Aux Controller Disconnected");
        }
        
        SmartDashboard.putNumber("Frozen_Dashboard_Detector_2000", i++);
        SmartDashboard.putNumber("BATTERY_voltage", RobotController.getBatteryVoltage());

        SmartDashboard.putBoolean("drive disabled", driveDisable.isOn());
        SmartDashboard.putBoolean("aux disabled", auxDisable.isOn());

        m_timeAndJoystickReplay.update();
        
        drivetrain.setFieldCentric(fieldCentric.isOn());
        drivetrain.setLimelightDisable(limelightDisable.isOn());
        
        conductor.periodic();
        blinkyBlinkyOrchestrator.sparkle();
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
    public void autonomousInit() {
        Command autoCommand = autoChooser.getSelectedCommand();
        if (autoCommand != Commands.none()) {
            CommandScheduler.getInstance().schedule(autoCommand);
        }
    }
    
    @Override
    public void autonomousPeriodic() {
        hubOrchestrator.runConvergance();
    }
    
    @Override
    public void autonomousExit() {}
    
    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {
        hubOrchestrator.runConvergance();
    }

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
