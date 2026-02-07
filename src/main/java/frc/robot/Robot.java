// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.orchestration.CannonOrchestrator;
import frc.robot.orchestration.HubOrchestrator;
import frc.robot.subsystems.Cannon.HoodSubsystem;
import frc.robot.subsystems.Cannon.ShooterSubsystem;
import frc.robot.subsystems.Cannon.TurretSubsystem;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Storage.KickerSubsystem;
import frc.robot.subsystems.Storage.SpindexerSubsystem;
import frc.util.Alert;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Constants.Swerve;

public class Robot extends TimedRobot {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController auxController = new CommandXboxController(1);

    // private final Telemetry logger = new Telemetry(Constants.SwerveConstants.kMaxSpeed);
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();
  
    public final SwerveSubsystem drivetrain = new SwerveSubsystem();

    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final HoodSubsystem hood = new HoodSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();
    
    public final SpindexerSubsystem spindexer = new SpindexerSubsystem();
    public final KickerSubsystem kicker = new KickerSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();

    public final CannonOrchestrator cannonOrchestrator;
    public final HubOrchestrator hubOrchestrator;

    public double shootSpeed = 0.2;

    public Robot() {
        // DataLogManager.start();
        Alert.info("The robot has restarted");

        driverController.leftTrigger(.1).onTrue(drivetrain.toggleFieldCentric());

        drivetrain.setDefaultCommand(
            drivetrain
                .driveWithJoysticks(driverController::getLeftX, driverController::getLeftY, driverController::getRightX)
        );
      
        RobotModeTriggers.disabled().whileTrue(drivetrain.idle());
        
        driverController.a().whileTrue(drivetrain.brick());
        driverController.b().whileTrue(drivetrain.pointWithController(driverController::getLeftX, driverController::getLeftY));

        // Reset the field-centric heading on left bumper press.
        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        driverController.rightBumper().onTrue(
            drivetrain.driveToPose()
                .withTarget(Swerve.targetPose)
        );

        driverController.y().whileTrue(drivetrain.driveLockedToArcWithJoysticks(driverController::getLeftX));

        // driverController.leftBumper().onTrue(shooter.turretToPosition(drivetrain::hubLockTurretAngle));
        driverController.rightBumper().onTrue(shooter.preheat()).onFalse(shooter.stopShooter()); // right bumper toggle shooter motor

        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysID.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysID.sysIdQuasistatic(Direction.kReverse));
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysID.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysID.sysIdDynamic(Direction.kReverse));
        // drivetrain.registerTelemetry(logger::telemeterize);}
        cannonOrchestrator = new CannonOrchestrator(this);
        hubOrchestrator = new HubOrchestrator(this);

        auxController.a().onTrue(kicker.playSoccer());
        auxController.a().onFalse(kicker.halt());

        auxController.a().onTrue(shooter.manual_shooter(() -> shootSpeed));
        auxController.a().onFalse(shooter.stopShooter());

        auxController.b().onTrue(new CommandBuilder()
            .onExecute(() -> {
                shootSpeed += .05;
            })
            .isFinished(true)
            .ignoringDisable(true)
        );
        auxController.x().onTrue(new CommandBuilder()
            .onExecute(() -> {
                shootSpeed -= .05;
            })
            .isFinished(true)
            .ignoringDisable(true)
        );
    }

    @Override
    public void robotPeriodic() {
        // DataLogManager.start();
        Alert.info(String.format("Last build time: %s, on branch %s", BuildConstants.BUILD_DATE, BuildConstants.GIT_BRANCH));

        m_timeAndJoystickReplay.update();
        SmartDashboard.putData(CommandScheduler.getInstance());
        SmartDashboard.putNumber("Speed", shootSpeed);
        SmartDashboard.putNumber("RPM", shootSpeed * Constants.Shooter.kTargetShooterRPM);
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
