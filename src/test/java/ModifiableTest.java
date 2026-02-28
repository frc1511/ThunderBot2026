import edu.wpi.first.hal.HAL;
import frc.robot.Robot;
import frc.robot.subsystems.Cannon.HoodSubsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ModifiableTest implements AutoCloseable {
  static Robot robot = null;

  @BeforeEach
  void robotBuilds() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    if (robot == null) {
      robot = new Robot();
      assertNotNull(robot);
    }
  }

  @Test
  void hoodZeroedExample() {
    HoodSubsystem hood = robot.hood;
    assertEquals(hood.getField("isConfirmedZeroed").getValue(), false);
    hood.getField("isConfirmedZeroed").withValue(() -> Boolean.TRUE);
    assertEquals(hood.isZeroed(), true);
  }

  @Override
  public void close() {
    robot.close();
  }
}
