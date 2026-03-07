// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.thunder.lib.auto.ThunderAutoProject;
import com.thunder.lib.auto.ThunderAutoSendableChooser;
import com.thunder.lib.trajectory.ThunderTrajectoryRunnerProperties;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        CommandScheduler.getInstance().registerSubsystem(kicker);
        CommandScheduler.getInstance().registerSubsystem(hood);
        CommandScheduler.getInstance().registerSubsystem(spindexer);
        CommandScheduler.getInstance().registerSubsystem(hang);
        DataLogManager.start(); //* Uncomment for logs
    
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
        drivetrain.setOptimalRotationGetter(hubOrchestrator::getOptimalDriveOrientation);
        
        kicker.getField("optimalRPM").withValue(hubOrchestrator::getOptimalShootSpeed);
        
        Alert.info("The robot has restarted");
        DriverStation.silenceJoystickConnectionWarning(true); // trying to fix radio problem
        SignalLogger.enableAutoLogging(true);
        
        ledDisable.get()
            .onTrue(
                new InstantCommand(() -> {
                    Broken.blinkyBlinkyButtonBopped = true;
                })
                .alongWith(
                    blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.OFF)
                )
            )
            .onFalse(
                new InstantCommand(() -> {
                    Broken.blinkyBlinkyButtonBopped = false;
                })
            );

        // MARK: Drive
        if (!Broken.drivetrainFullyDisabled) { // driving with joysticks
            drivetrain.setDefaultCommand(
                drivetrain
                    .driveWithJoysticks(
                        () -> {
                            if (driveDisable.isOff()) {
                                return driverController.getLeftX();
                            } else if (driveDisable.isOn() && auxDisable.isOff() && oneDriverMode.isOn()) {
                                return auxController.getLeftX();
                            }
                            return 0;
                        },
                        () -> {
                            if (driveDisable.isOff()) {
                                return driverController.getLeftY();
                            } else if (driveDisable.isOn() && auxDisable.isOff() && oneDriverMode.isOn()) {
                                return auxController.getLeftY();
                            }
                            return 0;
                        },
                        () -> {
                            if (driveDisable.isOff()) {
                                return driverController.getRightX();
                            } else if (driveDisable.isOn() && auxDisable.isOff() && oneDriverMode.isOn()) {
                                return auxController.getRightX();
                            }
                            return 0;
                        })
                    .onlyIf(() -> !driverController.y().getAsBoolean())
                    .withName("DriveWithJoysticks")
            );
        };

        RobotModeTriggers.disabled().onTrue(drivetrain.idle().withTimeout(.1));

        driverController.x().and(driveDisable::isOff).whileTrue(drivetrain.brick().withName("DriveBrick")); // polymorphs the robot into a brick (hold) upon release polymorphs the brick back into a robot
        driverController.y().and(driveDisable::isOff).whileTrue(drivetrain.hubLock().withName("DriveHubLockToggle")); // lock and shoot
        driverController.b().and(driveDisable::isOff).whileTrue(hang.jostle().withName("HangJostle"));
        driverController.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).withName("DriveSeedFieldCentric")); // reset IMU

        driverController.povUp()  .and(() -> driveDisable.isOff() && climberDisable.isOff()).whileTrue(hang.extend().withName("HangExtend")).onFalse(hang.halt().withName("HangHalt")); // hang go uppies (hold)
        driverController.povDown().and(() -> driveDisable.isOff() && climberDisable.isOff()).whileTrue(hang.retract().withName("HangRetract")).onFalse(hang.halt().withName("HangHalt")); // hang go downies (hold)
        driverController.povLeft().and(driveDisable::isOff).onTrue(drivetrain.increaseSpeed().withName("DriveSpeedInc")); // drive go weeee
        driverController.povRight().and(driveDisable::isOff).onTrue(drivetrain.decreaseSpeed().withName("DriveSpeedDesc")); // drive go snail

        fieldCentric.get()
            .onTrue(new InstantCommand(() -> drivetrain.setFieldCentric(true)))
            .onFalse(new InstantCommand(() -> drivetrain.setFieldCentric(false)));

        driverController.leftTrigger().and(driveDisable::isOff).whileTrue(drivetrain.toggleTrenchLock().withName("DriveTrenchLockToggle"));
        driverController.rightTrigger() // temporary robot centric
            .onTrue(
                new CommandBuilder()
                    .onExecute(() -> drivetrain.setFieldCentric(false))
                    .isFinished(true)
                    .onlyIf(() -> driveDisable.isOff() && oneDriverMode.isOff())
                    .withName("DriveRobotCentricSetOn")
            )
            .onFalse(
                new CommandBuilder()
                    .onExecute(() -> drivetrain.setFieldCentric(fieldCentric.isOn()))
                    .isFinished(true)
                    .withName("DriveRobotCentricSetOff")
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
                .withName("TrenchStow")
        );
        // MARK: One Driver Mode
        driverController.leftTrigger() // outtake (hold)
            .whileTrue(
                intake.outtake()
                .onlyIf(() -> driveDisable.isOff() && oneDriverMode.isOn())
                .withName("OneDriveIntakeOuttake")
            );
        driverController.leftBumper() // preheat (hold)
            .whileTrue(
                shooter.holdSpeedForShoot()
                .onlyIf(() -> driveDisable.isOff() && oneDriverMode.isOn())
                .withName("OneDriveShootPreheat")
            );
        driverController.rightTrigger() // intake (hold)
            .whileTrue(
                hungerOrchestrator.consume()
                .onlyIf(() -> driveDisable.isOff() && oneDriverMode.isOn())
                .withName("OneDriveHungerIntake")
            );
        driverController.rightBumper() // fire (hold)
            .whileTrue(
                firingOrchestrator.fire()
                .onlyIf(() -> driveDisable.isOff() && oneDriverMode.isOn())
                .withName("OneDriveFiringFire")
            );
        driverController.back()
            .onTrue(
                pivot.up()
                .onlyIf(() -> driveDisable.isOff() && oneDriverMode.isOn())
                .withName("OneDrivePivotUp")
            );

        driverController.start()
            .onTrue(
                hungerOrchestrator.jostle()
                .onlyIf(() -> driveDisable.isOff())
                .withName("HungerJostle")
            );

        // if drive is disabled and one driver mode is enabled
        auxController.leftTrigger() // outtake (hold)
            .whileTrue(
                intake.outtake()
                .onlyIf(() -> driveDisable.isOn() && oneDriverMode.isOn() && auxDisable.isOff())
                .withName("OneDriveBackupIntakeOuttake")
            );
        auxController.leftBumper() // preheat (hold)
            .whileTrue(
                shooter.holdSpeedForShoot()
                .onlyIf(() -> driveDisable.isOn() && oneDriverMode.isOn() && auxDisable.isOff())
                .withName("OneDriveBackupShooterPreheat")
            );
        auxController.rightTrigger() // intake (hold)
            .whileTrue(
                hungerOrchestrator.consume()
                .onlyIf(() -> driveDisable.isOn() && oneDriverMode.isOn() && auxDisable.isOff())
                .withName("OneDriveBackupHungerIntake")
            );
        auxController.rightBumper() // fire (hold)
            .whileTrue(
                firingOrchestrator.fire()
                .onlyIf(() -> driveDisable.isOn() && oneDriverMode.isOn() && auxDisable.isOff())
                .withName("OneDriveBackupFiringFire")
            );
        auxController.povUp().and(driveDisable::isOn).and(auxDisable::isOff).and(oneDriverMode::isOn).whileTrue(hang.extend().withName("OneDriveBackupHangExtend")).onFalse(hang.halt().withName("OneDriveBackupHangStop")); // hang go uppies (hold)
        auxController.povDown().and(driveDisable::isOn).and(auxDisable::isOff).and(oneDriverMode::isOn).whileTrue(hang.retract().withName("OneDriveBackupHangRetract")).onFalse(hang.halt().withName("OneDriveBackupHangStop")); // hang go downies (hold)
        auxController.povLeft().and(driveDisable::isOn).and(auxDisable::isOff).and(oneDriverMode::isOn).onTrue(drivetrain.increaseSpeed().withName("OneDriveBackupDriveIncSpeed")); // drive go weeee
        auxController.povRight().and(driveDisable::isOn).and(auxDisable::isOff).and(oneDriverMode::isOn).onTrue(drivetrain.decreaseSpeed().withName("OneDriveBackupDriveDescSpeed")); // drive go snail
        auxController.x().and(driveDisable::isOn).and(auxDisable::isOff).and(oneDriverMode::isOn).whileTrue(drivetrain.brick().withName("OneDriveBackupDriveBrick")); // polymorphs the robot into a brick (hold) upon release polymorphs the brick back into a robot
        auxController.y().and(driveDisable::isOn).and(auxDisable::isOff).and(oneDriverMode::isOn).whileTrue(drivetrain.hubLock().withName("OneDriveBackupDriveHubLock")); // lock and shoot
        auxController.b().and(driveDisable::isOn).and(auxDisable::isOff).and(oneDriverMode::isOn).whileTrue(hang.jostle().withName("OneDriveBackupJostle")); // lock and shoot
        auxController.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).onlyIf(() -> driveDisable.isOn() && oneDriverMode.isOn() && auxDisable.isOff()).withName("OneDriveBackupDriveSeedFieldCentric")); // reset IMU

        // MARK: Aux
        auxController.povUp() // Speen
            .whileTrue(
                new ParallelCommandGroup(
                    spindexer.spin(Constants.Storage.Spindexer.Duration.FOREVER)
                )
                .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
                .withName("SpindexerSpin")
            )
            .onFalse(spindexer.halt());

        auxController.y() // Feed
            .onTrue(
                hood.toPosition(() -> Constants.Hood.Position.FEED.get())
                .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
                .withName("PresetFeed")
            );

        auxController.x() // Tower
            .onTrue(
                hood.toPosition(() -> Constants.Hood.Position.TRENCH.get()) // TODO: evil trench please change this
                .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
                .withName("PresetTower")
            );

        auxController.b() // Trench
            .onTrue(
                hood.toPosition(() -> Constants.Hood.Position.TRENCH.get())
                .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
                .withName("PresetTrench")
            );

        auxController.a() // HUB
            .onTrue(
                hood.toPosition(() -> Constants.Hood.Position.HUB.get())
                .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
                .withName("PresetHub")
            );

        auxController.leftBumper() // Preheat (hold)
            .whileTrue(
                shooter.holdSpeedForShoot()
                .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
                .withName("ShooterPreheat")
            )
            .onFalse(shooter.halt().withName("ShooterHalt"));

        auxController.leftTrigger() // Fire (hold)
            .whileTrue(
                firingOrchestrator.fire()
                .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
                .withName("FiringFire")
            )
            .onFalse(firingOrchestrator.halt().withName("FiringHalt"));

        auxController.rightBumper() // Outtake (hold)
            .whileTrue(
                intake.outtake() 
                .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
                .withName("IntakeOuttake")
            )
            .onFalse(intake.stopEating().withName("IntakeHalt"));

        auxController.rightTrigger() // Intake (hold)
            .whileTrue(
                hungerOrchestrator.consume()
                .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
                .withName("HungerConsume")
            )
            .onFalse(intake.stopEating().withName("IntakeHalt"));

        auxController.back() // Intake (hold)
            .whileTrue(
                pivot.up()
                .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
                .withName("PivotUp")
            );

        auxController.start() // Hood down
            .onTrue(
                hood.toPosition(() -> Constants.Hood.Position.BOTTOM.get())
                .onlyIf(() -> auxDisable.isOff() && oneDriverMode.isOff())
                .withName("HoodBottom")
            );

        if (!Broken.hoodDisabled) hood.setDefaultCommand(hood.zero().withName("HoodHalt"));
        if (!Broken.shooterFullyDisabled) shooter.setDefaultCommand(shooter.halt().withName("ShooterHalt"));
        if (!Broken.kickerDisabled) kicker.setDefaultCommand(kicker.halt().withName("KickerHalt"));
        if (!Broken.hangFullyDisabled) hang.setDefaultCommand(hang.zeroHang().withName("HangZero"));

        // MARK: Auto
        ThunderAutoProject autoProject = AutoLoader.load(this);

        autoChooser = new ThunderAutoSendableChooser("Auto_Mode");

        autoChooser.includeProjectSource(autoProject);
        autoChooser.addAllAutoModesFromProject(autoProject.getName());
        // autoChooser.addTrajectoryFromProject(autoProject.getName(), "ShootFromStart");
        // autoChooser.addTrajectoryFromProject(autoProject.getName(), "test_teehee");
        // autoChooser.addTrajectoryFromProject(autoProject.getName(), "test_complex_shoot");
        ThunderTrajectoryRunnerProperties props = drivetrain.getTrajectoryRunnerProperties();
        if (props != null) {
            autoChooser.setTrajectoryRunnerProperties(drivetrain.getTrajectoryRunnerProperties());
        } else {
            Alert.error("Auto trajectory property fail");
        }

        // ShooterSysID shooterSysID = new ShooterSysID(shooter);
        // auxController.povUp().and(placeHolder9::isOn).whileTrue(shooterSysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // auxController.povRight().and(placeHolder9::isOn).whileTrue(shooterSysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // auxController.povDown().and(placeHolder9::isOn).whileTrue(shooterSysID.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // auxController.povLeft().and(placeHolder9::isOn).whileTrue(shooterSysID.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        SmartDashboard.putData(CommandScheduler.getInstance());

        SmartDashboard.putData("SwitchBoard", switchBoard);
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

        // m_timeAndJoystickReplay.update();
        
        drivetrain.setLimelightDisable(limelightDisable.isOn());
        
        conductor.periodic();
        blinkyBlinkyOrchestrator.sparkle();
        CommandScheduler.getInstance().run();
    }
    
    @Override
    public void disabledInit() {
        pivot.setMotorMode(IdleMode.kCoast);
        CommandScheduler.getInstance().cancelAll();
    }
    
    @Override
    public void disabledPeriodic() {}
    
    @Override
    public void disabledExit() {
        pivot.setMotorMode(IdleMode.kBrake);
    }
    
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
    public void autonomousExit() {
        CommandScheduler.getInstance().schedule(drivetrain.setHubLock(false).ignoringDisable(true));
        intake.stopEating().execute();
        // CommandScheduler.getInstance().schedule(intake.stopEating().ignoringDisable(true));
    }
    
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
