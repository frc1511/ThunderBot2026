// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashSet;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.orchestration.BlinkyBlinkyOrchestrator;
import frc.robot.orchestration.CannonOrchestrator;
import frc.robot.orchestration.Conductor;
import frc.robot.orchestration.FiringOrchestrator;
import frc.robot.orchestration.HubOrchestrator;
import frc.robot.orchestration.HungerOrchestrator;
import frc.robot.subsystems.Cannon.HoodSubsystem;
import frc.robot.subsystems.Cannon.ShooterSubsystem;
import frc.robot.subsystems.Cannon.ShooterSysID;
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
import frc.util.Helpers;
import frc.util.Thunder.ThunderInterface;
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

    private final SendableChooser<Command> autoChooser;

    private Timer hubActiveTimer = new Timer();

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

    public PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);

    public HashSet<ThunderInterface> allSubsystems = new HashSet<>();

    public Robot() {
        allSubsystems.add(shooter);
        allSubsystems.add(hood);
        allSubsystems.add(turret);
        allSubsystems.add(spindexer);
        allSubsystems.add(kicker);
        allSubsystems.add(intake);
        allSubsystems.add(pivot);
        allSubsystems.add(hang);
        allSubsystems.add(drivetrain);

        CommandScheduler.getInstance().registerSubsystem(drivetrain);
        allSubsystems.forEach(
            subsystem -> {
                CommandScheduler.getInstance().registerSubsystem(subsystem);
            }
        );

        if (Constants.kUseDataLog) {
            DataLogManager.start();
        }

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
        SignalLogger.enableAutoLogging(Constants.kUseSignalLogger);

        { // Default Commands
            if (!Broken.hoodDisabled) hood.setDefaultCommand(hood.zero().withName("HoodHalt"));
            if (!Broken.shooterFullyDisabled) shooter.setDefaultCommand(shooter.halt().withName("ShooterHalt"));
            if (!Broken.kickerDisabled) kicker.setDefaultCommand(kicker.halt().withName("KickerHalt"));
            if (!Broken.hangFullyDisabled) hang.setDefaultCommand(hang.zeroHang().withName("HangZero"));
            if (!Broken.drivetrainFullyDisabled) { // driving with joysticks
                BooleanSupplier onedriveAuxFallback = () -> driveDisable.isOn() && auxDisable.isOff() && oneDriverMode.isOn();

                drivetrain.setDefaultCommand(
                    drivetrain
                        .driveWithJoysticks(
                            () -> {
                                if (driveDisable.isOff()) {
                                    return driverController.getLeftX();
                                } else if (onedriveAuxFallback.getAsBoolean()) {
                                    return auxController.getLeftX();
                                }
                                return 0;
                            },
                            () -> {
                                if (driveDisable.isOff()) {
                                    return driverController.getLeftY();
                                } else if (onedriveAuxFallback.getAsBoolean()) {
                                    return auxController.getLeftY();
                                }
                                return 0;
                            },
                            () -> {
                                if (driveDisable.isOff()) {
                                    return driverController.getRightX();
                                } else if (onedriveAuxFallback.getAsBoolean()) {
                                    return auxController.getRightX();
                                }
                                return 0;
                            })
                        .onlyIf(() -> !driverController.y().getAsBoolean())
                        .withName("DriveWithJoysticks")
                );
            };

            RobotModeTriggers.disabled().onTrue(drivetrain.idle().withTimeout(.1));
        }
        
        { // Switchboard
            ledDisable.get()
                .whileTrue(
                    new InstantCommand(() -> {
                        Broken.blinkyBlinkyButtonBopped = true;
                    })
                    .beforeStarting(
                        blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.OFF)
                    ).ignoringDisable(true).repeatedly()
                )
                .onFalse(
                    new InstantCommand(() -> {
                        Broken.blinkyBlinkyButtonBopped = false;
                    }).ignoringDisable(true)
                );
    
            pitMode.get()
                .whileTrue(blinkyBlinkyOrchestrator.set(Constants.BlinkyBlinky.Mode.PIT).ignoringDisable(true));
    
            fieldCentric.get()
                .onTrue(new InstantCommand(() -> drivetrain.setFieldCentric(true)))
                .onFalse(new InstantCommand(() -> drivetrain.setFieldCentric(false)));
        }
        
        /**********************/
        // MARK: Drive
        /**********************/

        { // Omnipresent Drive Controls (OneDrive Off/On)
            BooleanSupplier condition = () -> driveDisable.isOff(); // Always present on drive controller

            driverController.x().and(condition).whileTrue(drivetrain.brick().withName("DriveBrick")); // polymorphs the robot into a brick (hold) upon release polymorphs the brick back into a robot
            driverController.y().and(condition).whileTrue(drivetrain.hubLock().withName("DriveHubLockToggle")); // lock and shoot
            driverController.a().and(condition).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).withName("DriveSeedFieldCentric")); // reset IMU
        }

        { // Hang Controls
            BooleanSupplier condition = () -> driveDisable.isOff() && climberDisable.isOff();

            driverController.b().and(condition).whileTrue(hang.jostle().withName("HangJostle"));
            driverController.povUp().and(condition).whileTrue(hang.extend().withName("HangExtend")).onFalse(hang.halt().withName("HangHalt")); // hang go uppies (hold)
            driverController.povDown().and(condition).whileTrue(hang.retract().withName("HangRetract")).onFalse(hang.halt().withName("HangHalt")); // hang go downies (hold)
        }

        { // Default Drive Controls (OneDrive Off)
            BooleanSupplier condition = () -> driveDisable.isOff() && oneDriverMode.isOff();

            driverController.leftTrigger() .and(condition).whileTrue(drivetrain.trenchLock().withName("DriveTrenchLockToggle"));
            driverController.leftBumper()  .and(condition).onTrue(drivetrain.decreaseSpeed().withName("DriveSpeedDesc")); // Drive go snail
            driverController.rightBumper() .and(condition).onTrue(drivetrain.increaseSpeed().withName("DriveSpeedInc")); // Drive go weeee
            driverController.rightTrigger().and(condition) // Temporary robot centric
                .onTrue(
                    new CommandBuilder()
                        .onExecute(() -> drivetrain.setFieldCentric(false))
                        .isFinished(true)
                        .withName("DriveRobotCentricSetOn")
                )
                .onFalse(
                    new CommandBuilder()
                        .onExecute(() -> drivetrain.setFieldCentric(fieldCentric.isOn()))
                        .isFinished(true)
                        .withName("DriveRobotCentricSetOff")
                );

            // Puts the hang and hood in down mode when going under trench
            driverController.start().and(condition).whileTrue(
                hang.stowForTrench()
                    .alongWith(
                        hood.stowForTrench()
                    )
                    .withName("TrenchStow")
            );
        }

        { // OneDrive Drive Controls (OneDrive On)
            BooleanSupplier condition = () -> driveDisable.isOff() && oneDriverMode.isOn();

            driverController.povLeft().and(condition).onTrue(drivetrain.decreaseSpeed().withName("OneDriveDriveDescSpeed")); // drive go weeee
            driverController.povRight().and(condition).onTrue(drivetrain.increaseSpeed().withName("OneDriveDriveIncSpeed")); // drive go snail

            driverController.leftTrigger() .and(condition).whileTrue(intake.outtake().withName("OneDriveIntakeOuttake"));
            driverController.leftBumper()  .and(condition).whileTrue(shooter.holdSpeedForShoot().withName("OneDriveShootPreheat"));
            driverController.rightTrigger().and(condition).whileTrue(hungerOrchestrator.consume().withName("OneDriveHungerIntake"));
            driverController.rightBumper() .and(condition).whileTrue(firingOrchestrator.fire().withName("OneDriveFiringFire"));
            driverController.back()        .and(condition).onTrue(pivot.up().withName("OneDrivePivotUp"));
        }

        /**********************/
        // MARK: Aux
        /**********************/

        { // OneDrive Aux Fallback
            BooleanSupplier condition = () -> driveDisable.isOn() && oneDriverMode.isOn() && auxDisable.isOff();

            auxController.leftTrigger() // outtake (hold)
                .and(condition)
                .whileTrue(
                    intake.outtake()
                    .withName("OneDriveBackupIntakeOuttake")
                );
            auxController.leftBumper() // preheat (hold)
                .and(condition)
                .whileTrue(
                    shooter.holdSpeedForShoot()
                    .withName("OneDriveBackupShooterPreheat")
                );
            auxController.rightTrigger() // intake (hold)
                .and(condition)
                .whileTrue(
                    hungerOrchestrator.consume()
                    .withName("OneDriveBackupHungerIntake")
                );
            auxController.rightBumper() // fire (hold)
                .and(condition)
                .whileTrue(
                    firingOrchestrator.fire()
                    .withName("OneDriveBackupFiringFire")
                );

            auxController.b().and(condition)      .whileTrue(hang.jostle()             .withName("OneDriveBackupJostle"));
            auxController.x().and(condition)      .whileTrue(drivetrain.brick()        .withName("OneDriveBackupDriveBrick")); // polymorphs the robot into a brick (hold) upon release polymorphs the brick back into a robot
            auxController.y().and(condition)      .whileTrue(drivetrain.hubLock()      .withName("OneDriveBackupDriveHubLock")); // lock and shoot
            auxController.a().and(condition)         .onTrue(drivetrain.resetRotation().withName("OneDriveBackupDriveSeedFieldCentric")); // reset IMU
            auxController.povLeft().and(condition)   .onTrue(drivetrain.decreaseSpeed().withName("OneDriveBackupDriveDescSpeed")); // drive go weeee
            auxController.povRight().and(condition)  .onTrue(drivetrain.increaseSpeed().withName("OneDriveBackupDriveIncSpeed")); // drive go snail
            auxController.povUp().and(condition)  .whileTrue(hang.extend()             .withName("OneDriveBackupHangExtend")) // hang go uppies (hold)
                                                    .onFalse(hang.halt()               .withName("OneDriveBackupHangStop"));
            auxController.povDown().and(condition).whileTrue(hang.retract()            .withName("OneDriveBackupHangRetract")) // hang go downies (hold)
                                                    .onFalse(hang.halt()               .withName("OneDriveBackupHangStop"));
        }

        { // Main Aux Controls
            BooleanSupplier condition = () -> auxDisable.isOff() && oneDriverMode.isOff();

            (auxController.leftStick().or(auxController.rightStick())).and(condition)
                                                       .whileTrue(shooter.reverse()                                           .withName("ShooterReverse"));
            auxController.povUp().and(condition)       .whileTrue(spindexer.spin(Constants.Storage.Spindexer.Duration.FOREVER).withName("SpindexerSpin"))
                                                         .onFalse(spindexer.halt()                                            .withName("SpindexerHalt"));
            auxController.start().and(condition)       .whileTrue(hungerOrchestrator.jostleRepeatedly()                       .withName("HungerJostle"));
            auxController.y().and(condition)           .whileTrue(hood.toPosition(() -> Constants.Hood.Position.FEED.get())   .withName("PresetFeed"))
                                                         .onFalse(hood.toPosition(() -> Constants.Hood.Position.TRENCH.get()) .withName("PresetPostFeed"));
            auxController.x().and(condition)              .onTrue(hood.toPosition(() -> Constants.Hood.Position.TRENCH.get()) .withName("PresetTower"));
            auxController.b().and(condition)              .onTrue(hood.toPosition(() -> Constants.Hood.Position.TRENCH.get()) .withName("PresetTrench"));
            auxController.a().and(condition)              .onTrue(hood.toPosition(() -> Constants.Hood.Position.HUB.get())    .withName("PresetHub"));
            auxController.leftBumper().and(condition)  .whileTrue(shooter.holdSpeedForShoot()                                 .withName("ShooterPreheat"))
                                                         .onFalse(shooter.halt()                                              .withName("ShooterHalt"));
            auxController.leftTrigger().and(condition) .whileTrue(firingOrchestrator.fire()                                   .withName("FiringFire"))
                                                         .onFalse(firingOrchestrator.halt()                                   .withName("FiringHalt"));
            auxController.rightBumper().and(condition) .whileTrue(intake.outtake()                                            .withName("IntakeOuttake"))
                                                         .onFalse(intake.stopEating()                                         .withName("IntakeHalt"));
            auxController.rightTrigger().and(condition).whileTrue(hungerOrchestrator.consume()                                .withName("HungerConsume"))
                                                         .onFalse(intake.stopEating()                                         .withName("IntakeHalt"));
            auxController.back().and(condition)        .whileTrue(pivot.up()                                                  .withName("PivotUp"));
        }
        
        // ShooterSysID shooterSysID = new ShooterSysID(shooter);
        // auxController.povUp().and(placeHolder9::isOn).whileTrue(shooterSysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // auxController.povRight().and(placeHolder9::isOn).whileTrue(shooterSysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // auxController.povDown().and(placeHolder9::isOn).whileTrue(shooterSysID.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // auxController.povLeft().and(placeHolder9::isOn).whileTrue(shooterSysID.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // MARK: Auto
        NamedCommands.registerCommand("DB_Hub_AL_Start", drivetrain.setHubLock(true));
        NamedCommands.registerCommand("DB_Hub_AL_Stop", drivetrain.setHubLock(false));
        NamedCommands.registerCommand("Preheat", shooter.holdSpeedForShoot().withTimeout(0));
        NamedCommands.registerCommand("Shoot", firingOrchestrator.fireThenStop().alongWith(hungerOrchestrator.jostleRepeatedly()).withTimeout(3));
        NamedCommands.registerCommand("AutoHang", conductor.autoHang());
        NamedCommands.registerCommand("Intake", intake.eatStart());
        NamedCommands.registerCommand("StopIntake", intake.stopEating());
        NamedCommands.registerCommand("StopShoot", firingOrchestrator.halt());
        NamedCommands.registerCommand("PivotDown", pivot.down());
        NamedCommands.registerCommand("PivotUp", pivot.up());
        NamedCommands.registerCommand("PivotDownSoft", pivot.downSoft());
        NamedCommands.registerCommand("PivotUpSoft", pivot.upSoft());
        NamedCommands.registerCommand("HoodDown", hood.toPosition(Constants.Hood.Position.BOTTOM::get));
        NamedCommands.registerCommand("ShootForever", firingOrchestrator.fireThenStop().alongWith(hungerOrchestrator.jostleRepeatedly()).withTimeout(20).until(RobotModeTriggers.disabled()::getAsBoolean));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);


        SmartDashboard.putData(CommandScheduler.getInstance());

        SmartDashboard.putData("SwitchBoard", switchBoard);

        Helpers.hubToBecomeActive().onTrue(new InstantCommand(() -> {
            hubActiveTimer.restart();
        }));

        if (Constants.kUseHDDL) {
            addPeriodic(() -> {
                allSubsystems.forEach(subsystem -> {
                    subsystem.hddlPeriodic();
                });
            }, 1d/Constants.kHDDLRate, .005d);
        }
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
        SmartDashboard.putBoolean("drive disabled", driveDisable.isOn());
        SmartDashboard.putBoolean("aux disabled", auxDisable.isOn());
        m_timeAndJoystickReplay.update();

        SmartDashboard.putNumber("Frozen_Dashboard_Detector_2000", i++);

        SmartDashboard.putString("aid_hubTimer", String.format("%.3f", Math.max(25 - hubActiveTimer.get(), -2.0)).replace(".", "s"));
        SmartDashboard.putBoolean("aid_hubActive", Helpers.isHubActive(true));
        SmartDashboard.putNumber("aid_matchTimer", Timer.getMatchTime());

        SmartDashboard.putBoolean("aid_tenchMode", conductor.trenchSafe());

        // If the HDDL is not running, run them at the normal rate
        if (!Constants.kUseHDDL) {
            allSubsystems.forEach(ThunderInterface::hddlPeriodic);
        }

        drivetrain.setLimelightDisable(limelightDisable.isOn());
        
        conductor.periodic();
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
        hubActiveTimer.start();
        drivetrain.setupTheta(true);
        drivetrain.setSpeedMultiplier(1.0);
        drivetrain.setupTheta(true);
        Command autoCommand = autoChooser.getSelected();
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
        drivetrain.setupTheta(false);
        drivetrain.setHubLock(false).execute();;
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
