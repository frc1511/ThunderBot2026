package frc.util;

import java.util.ArrayList;
import java.util.List;

public class FiringTable {
    public class FiringDataPoint {
        public double distance;
        public double speedRPM;
        public double hoodAngle;
        public double timeOfFlight;

        FiringDataPoint(double distance, double speedRPM, double hoodAngle, double timeOfFlight) {
            this.distance = distance;
            this.speedRPM = speedRPM;
            this.hoodAngle = hoodAngle;
            this.timeOfFlight = timeOfFlight;
        }
    }

    public List<FiringDataPoint> firingTable = new ArrayList<FiringDataPoint>();

    public FiringTable() {
        // TOF fudged bc we dont need it rn
        firingTable.add(new FiringDataPoint(2.169, 1700, .05, .7));
        firingTable.add(new FiringDataPoint(2.476, 1750, .1, .8));
        firingTable.add(new FiringDataPoint(3.000, 1840, 0.11, 0.86));
        firingTable.add(new FiringDataPoint(3.465, 1880, .12, .945));
        firingTable.add(new FiringDataPoint(3.715, 1965, .13, .98));
        firingTable.add(new FiringDataPoint(3.790, 1970, 0.14, 1));
        firingTable.add(new FiringDataPoint(4.021, 1990, 0.16, 1.0174));
        firingTable.add(new FiringDataPoint(4.92, 2120, 0.175, 1.1875));

        // firingTable.add(new FiringDataPoint(2.99, 1880, 0.25, 0.86));
        // firingTable.add(new FiringDataPoint(3.76, 1980, 0.245, 1.0174));
        // firingTable.add(new FiringDataPoint(4.95, 2080, 0.241, 1.1875));
    }

    public FiringDataPoint lerp(double distance) {
        FiringDataPoint lastPoint = firingTable.get(0);
        FiringDataPoint currentPoint = firingTable.get(0);
        for (FiringDataPoint currentPoint_ : firingTable) {
            lastPoint = currentPoint;
            currentPoint = currentPoint_;
            if (lastPoint.distance < distance && distance <= currentPoint_.distance) {
                break;
            }
        }

        double lerpPercentage = (distance - lastPoint.distance) / (currentPoint.distance - lastPoint.distance); // How far the distance is past the last point divided by how far it is between the 2 points

        return new FiringDataPoint(
            distance, 
            Helpers.lerp(lastPoint.speedRPM, currentPoint.speedRPM, lerpPercentage),
            Helpers.lerp(lastPoint.hoodAngle, currentPoint.hoodAngle, lerpPercentage),
            Helpers.lerp(lastPoint.timeOfFlight, currentPoint.timeOfFlight, lerpPercentage)
        );
    }
}
