import edu.wpi.first.hal.HAL;
import frc.util.Thunder.Modifiable;
import frc.util.Thunder.ThunderSubsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ModifiableTest {
  public class DummySubsystem extends ThunderSubsystem {
    public DummySubsystem() {
      new Modifiable("test", this, () -> Integer.valueOf(0));
    }

    public int get() {
      Modifiable field = getField("test");
      if (field != null && field.getValue() instanceof Integer) return (Integer) field.getValue();
      return 0;
    }
  }

  DummySubsystem dummy = null;

  @BeforeEach
  void robotBuilds() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    if (dummy == null) {
      dummy = new DummySubsystem();
      assertNotNull(dummy);
    }
  }

  @Test
  void hoodZeroedExample() {
    assertEquals(dummy.getField("test").getValue(), 0);
    dummy.getField("test").withValue(() -> Integer.valueOf(5));
    assertEquals(dummy.get(), 5);
  }
}
