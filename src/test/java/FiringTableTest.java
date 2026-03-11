import frc.util.FiringTable;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class FiringTableTest {
  static FiringTable firingTable = null;

  @BeforeEach
  void firingTableBuilds() {
    firingTable = new FiringTable();
  }

  @Test
  void enoughPoints() {
    assertNotNull(firingTable);
    assertTrue(firingTable.firingTable.size() >= 2);
  }
}
