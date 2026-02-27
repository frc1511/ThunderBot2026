import static org.junit.jupiter.api.Assertions.assertNotEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.util.ZoneConstants;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ZoneTest {
  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
  }

  @Test
  void testZones() {
    for (int i = 0; i < 0x0F; i++) {
      boolean isBump =      (i & 0b0001) != 0;
      boolean isBlueSide =  (i & 0b0010) != 0;
      boolean isDSside =    (i & 0b0100) != 0;
      boolean isRightSide = (i & 0b1000) != 0;

      Rectangle2d rect = ZoneConstants.constructZone(isBump, isBlueSide, isDSside, isRightSide);
      System.out.println(String.format("+ Bump:%b, BlueSide:%b, DSside:%b, RightSide:%b \n| ", isBump, isBlueSide, isDSside, isRightSide) + rect.toString());
      
      assert rect.getCenter().getX() > 0;
      assert rect.getCenter().getY() > 0;
      assert rect.getXWidth() > 0;
      assert rect.getYWidth() > 0;
      assertNotEquals(rect, new Rectangle2d(Translation2d.kZero, Translation2d.kZero), "Zone is zero! Invalid Zone");
    }
  }
}
