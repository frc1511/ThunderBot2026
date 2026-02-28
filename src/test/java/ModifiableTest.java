import edu.wpi.first.hal.HAL;
import frc.robot.Robot;
import frc.robot.subsystems.Cannon.HoodSubsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ModifiableTest {
  static HoodSubsystem hood = null;

  @BeforeEach
  void robotBuilds() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    if (hood == null) {
      hood = new HoodSubsystem();
      assertNotNull(hood);
    }
  }

  @Test
  void hoodZeroedExample() {
    assertEquals(hood.getField("isConfirmedZeroed").getValue(), false);
    hood.getField("isConfirmedZeroed").withValue(() -> Boolean.TRUE);
    assertEquals(hood.isZeroed(), true);
  }
}
