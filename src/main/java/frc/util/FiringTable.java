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
        firingTable.add(new FiringDataPoint(2.99, 1830, 0.25, 0.86));
        firingTable.add(new FiringDataPoint(3.76, 1900, 0.2, 0.9174));
        firingTable.add(new FiringDataPoint(4.95, 2100, 0.241, 1.1875));
        
        // firingTable.add(new FiringDataPoint(2.99, 1830, 0.25, 0.86));
        // firingTable.add(new FiringDataPoint(3.76, 1900, 0.2, 0.9174));
        // firingTable.add(new FiringDataPoint(4.95, 2100, 0.241, 1.1875));

        // firingTable.add(new FiringDataPoint(3, 1800, 0.25, 1));
        // firingTable.add(new FiringDataPoint(3.5, 1900, 0.26, 1.2));
        // firingTable.add(new FiringDataPoint(4, 2000, 0.27, 1.3));
        // firingTable.add(new FiringDataPoint(4.5, 2100, 0.28, 1.35));
        // firingTable.add(new FiringDataPoint(5, 2200, 0.29, 1.38));
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
