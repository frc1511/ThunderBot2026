package frc.util;

import java.util.ArrayList;
import java.util.List;

public class StaticFiringTable {
    public class StaticFiringDataPoint extends FiringDataPoint {
        public double distance;
        public String name;
        public double speedRPM;
        public double hoodAngle;
        public double timeOfFlight;

        StaticFiringDataPoint(String name, double speedRPM, double hoodAngle) {
            super(-1, speedRPM, hoodAngle, -1);
            this.name = name;
        }
    }

    public List<StaticFiringDataPoint> staticFiringTable = new ArrayList<StaticFiringDataPoint>();

    public StaticFiringTable() {
        staticFiringTable.add(new StaticFiringDataPoint("trench", 2015, 0.047d));
        staticFiringTable.add(new StaticFiringDataPoint("tower", 1976, 0.043d));
    }
}
