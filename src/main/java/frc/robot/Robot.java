// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Drive.SwerveSubsystem;

public class Robot extends TimedRobot {

  private final CommandXboxController driverController = new CommandXboxController(0);

  // private final Telemetry logger = new Telemetry(Constants.SwerveConstants.kMaxSpeed);
  private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
      .withTimestampReplay()
      .withJoystickReplay();
  public final SwerveSubsystem drivetrain = new SwerveSubsystem();

  public Robot() {
    DataLogManager.start();

    final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    driverController.leftTrigger(.1).onTrue(drivetrain.toggleFieldCentric());

    drivetrain.setDefaultCommand(
        drivetrain.driveWithJoysticks(driverController.getLeftX(), driverController.getLeftY(), driverController.getRightX())
    );

    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.b().whileTrue(drivetrain.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
    ));

    // Reset the field-centric heading on left bumper press.
    driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    Pose2d targetPose = Pose2d.kZero;
    Rotation2d targetRotation = Rotation2d.kZero;
    driverController.rightBumper().onTrue(drivetrain.driveToPose(targetPose, targetRotation, 0.0d));

    //* This should probably stay separate from the rest of controls
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driverController.start().and(driverController.y()).whileTrue(drivetrain.sysID.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x()).whileTrue(drivetrain.sysID.sysIdQuasistatic(Direction.kReverse));
    driverController.back().and(driverController.y()).whileTrue(drivetrain.sysID.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x()).whileTrue(drivetrain.sysID.sysIdDynamic(Direction.kReverse));
    // drivetrain.registerTelemetry(logger::telemeterize);
}

  @Override
  public void robotPeriodic() {
    m_timeAndJoystickReplay.update();
    SmartDashboard.putData(CommandScheduler.getInstance());

    if (!drivetrain.isCANSafe()) {
      CommandScheduler.getInstance().disable();
      return;
    }
    CommandScheduler.getInstance().enable();
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
  public void teleopInit() {
  }

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
