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
        firingTable.add(new FiringDataPoint(1.722, 1730, 0.01, 0.76));
        firingTable.add(new FiringDataPoint(2.231, 1800, 0.25, 0.82));
        firingTable.add(new FiringDataPoint(2.804, 1950, 0.04, 0.99));
        firingTable.add(new FiringDataPoint(3.628, 2040, 0.05, 1.11));
        firingTable.add(new FiringDataPoint(4.037, 2170, 0.06, 1.21));
        firingTable.add(new FiringDataPoint(4.411, 2180, 0.07, 1.24));
        firingTable.add(new FiringDataPoint(4.955, 2240, 0.08, 1.29));
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

        if (lerpPercentage < 0d) {
            return new FiringDataPoint(currentPoint.distance, currentPoint.speedRPM, currentPoint.hoodAngle, currentPoint.timeOfFlight);
        }

        return new FiringDataPoint(
            distance, 
            Helpers.lerp(lastPoint.speedRPM, currentPoint.speedRPM, lerpPercentage),
            Helpers.lerp(lastPoint.hoodAngle, currentPoint.hoodAngle, lerpPercentage),
            Helpers.lerp(lastPoint.timeOfFlight, currentPoint.timeOfFlight, lerpPercentage)
        );
    }
}
