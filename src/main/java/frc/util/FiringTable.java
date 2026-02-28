package frc.util;

import java.util.ArrayList;
import java.util.List;

public class FiringTable {
    public class FiringDataPoint {
        double distance;
        double speedRPM;
        double hoodAngle;

        FiringDataPoint(double distance, double speedRPM, double hoodAngle) {
            this.distance = distance;
            this.speedRPM = speedRPM;
            this.hoodAngle = hoodAngle;
        }
    }

    private List<FiringDataPoint> firingTable = new ArrayList<FiringDataPoint>();

    FiringTable() {
        firingTable.add(new FiringDataPoint(0.1, 1800, 0));
        firingTable.add(new FiringDataPoint(1.0, 2000, 0.2));
        firingTable.add(new FiringDataPoint(2.0, 2000, 0.2));
    }

    // private FiringDataPoint lerp(double distance) {
    //     return new FiringDataPoint(distance, null, null);
    // }
}
