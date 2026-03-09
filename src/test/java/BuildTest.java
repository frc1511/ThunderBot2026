import edu.wpi.first.hal.HAL;
import frc.robot.Robot;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class BuildTest {
  static Robot robot = null;

  @BeforeEach
  void robotBuilds() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    if (robot == null) {
      try {
        robot = new Robot();
      } catch (Exception e) {
        e.printStackTrace();
      assertNotNull(robot);
    }
    }
  }

  @Test
  void nonNullSubsystems() {
    assertNotNull(robot);
    assertNotNull(robot.drivetrain);
    assertNotNull(robot.shooter);
    assertNotNull(robot.hood);
    assertNotNull(robot.turret);
    assertNotNull(robot.spindexer);
    assertNotNull(robot.intake);
    assertNotNull(robot.pivot);
    assertNotNull(robot.hang);
  }

  @Test
  void init() {
    assertNotNull(robot);
    robot.robotInit();
    robot.disabledInit();
    robot.autonomousInit();
    robot.teleopInit();
  }
  
  @Test
  void periodic() {
    assertNotNull(robot);
    robot.robotPeriodic();
    robot.disabledPeriodic();
    robot.autonomousPeriodic();
    robot.teleopPeriodic();
  }
}
