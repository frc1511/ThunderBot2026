// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.thunder.lib.auto.ThunderAutoProject;
import com.thunder.lib.auto.ThunderAutoSendableChooser;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.orchestration.Conductor;
import frc.robot.orchestration.HubOrchestrator;
import frc.robot.orchestration.Autonomous.AutoLoader;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.util.Alert;
import frc.util.Broken;
import frc.util.Constants.Swerve;

public class Robot extends TimedRobot {

    private final CommandXboxController driverController = new CommandXboxController(0);

    // private final Telemetry logger = new Telemetry(Constants.SwerveConstants.kMaxSpeed);
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();
  
    public final SwerveSubsystem drivetrain = Broken.drivetrain ? null : new SwerveSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();

    private final HubOrchestrator hubOrchestrator = new HubOrchestrator(
        shooter
    );

    private final Conductor conductor = new Conductor(hubOrchestrator);

    ThunderAutoSendableChooser autoChooser;

    public Robot() {
        // DataLogManager.start();
        Alert.info("The robot has restarted");

        driverController.leftTrigger(.1).onTrue(drivetrain.toggleFieldCentric());

        drivetrain.setDefaultCommand(
            drivetrain
                .driveWithJoysticks(driverController::getLeftX, driverController::getLeftY, driverController::getRightX).onlyIf(() -> !driverController.y().getAsBoolean())
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
        // driverController.rightBumper().onTrue(shooter.shoot()).onFalse(shooter.stopShoot()); // right bumper toggle shooter motor

        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysID.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysID.sysIdQuasistatic(Direction.kReverse));
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysID.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysID.sysIdDynamic(Direction.kReverse));
        // drivetrain.registerTelemetry(logger::telemeterize);

        ThunderAutoProject autoProject = AutoLoader.load(conductor);

        autoChooser = new ThunderAutoSendableChooser("Auto_Mode");

        autoChooser.includeProjectSource(autoProject);
        autoChooser.addAllAutoModesFromProject(autoProject.getName());
        autoChooser.addTrajectoryFromProject(autoProject.getName(), "ShootFromStart");
        autoChooser.addTrajectoryFromProject(autoProject.getName(), "teehee");
        autoChooser.setTrajectoryRunnerProperties(drivetrain.getTrajectoryRunnerProperties());
    }

    @Override
    public void robotInit() {
        Alert.info(String.format("Last build time: %s, on branch %s.", BuildConstants.BUILD_DATE, BuildConstants.GIT_BRANCH) + (BuildConstants.DIRTY == 1 ? "Modified" : ""));
    }

    @Override
    public void robotPeriodic() {
        if (!driverController.isConnected()) {
            Alert.error("Drive Controller Disconnected");
        }

        if (RobotController.getBatteryVoltage() < 9) {
            Alert.critical(String.format("Battery voltage dipped below 9v, reached %.2f", RobotController.getBatteryVoltage()));
        } else if (RobotController.getBatteryVoltage() < 10) {
            Alert.error("Battery voltage dipped below 10v");
        }

        m_timeAndJoystickReplay.update();
        SmartDashboard.putData(CommandScheduler.getInstance());

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
