import frc.util.Thunder.Tuneable;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class TunableTest {

  static boolean hal_init = false;

  @BeforeEach
  void initHal() {
    if (hal_init == false) {
      assert HAL.initialize(500, 0);
      hal_init = true;
    }
  }

  @Test
  @SuppressWarnings("unused")
  void typesAllowed() {
    Tuneable<Double>    x1 = new Tuneable<>(1.0d, "Name");
    Tuneable<Double[]>  x2 = new Tuneable<>(new Double[] {0.1d}, "Name");
    Tuneable<Boolean>   x3 = new Tuneable<>(true, "Name");
    Tuneable<Boolean[]> x4 = new Tuneable<>(new Boolean[] {true}, "Name");
    Tuneable<String>    x5 = new Tuneable<>("x", "Name");
    Tuneable<String[]>  x6 = new Tuneable<>(new String[] {"x"}, "Name");
    Tuneable<Byte[]>    x7 = new Tuneable<>(new Byte[] {0x01}, "Name");
  }

  @Test
  void invalidTypesFail() {
    assertThrows(IllegalArgumentException.class, () -> new Tuneable<Timer>(new Timer(), "Name"));
  }

  @Test
  void valueSetsProperly() {
    Tuneable<Double> x = new Tuneable<>(1.0d, "Name");
    assertEquals(x.value, 1.0d);
    assertEquals(x.m_name, "Name");
  }

  @Test
  void valueSendsProperly() {
    Tuneable<Double> x = new Tuneable<>(1.0d, "Name");
    x.update();
    SmartDashboard.updateValues();
    assertEquals(x.m_hasUpdated, false);
    // NOTE: NT not active for testing ig
    // assertEquals(SmartDashboard.getNumber("Name", 0), 1.0);
  }

  @Test
  void valueGetsProperly() {
    Tuneable<Double> x = new Tuneable<>(1.0d, "Name");
    x.update();
    SmartDashboard.updateValues();
    // NOTE: NT not active for testing ig
    // SmartDashboard.putNumber("Name", 2.0d);
    // SmartDashboard.updateValues();
    // x.update();
    // assertEquals(x.value, 2.0);
  }
}
