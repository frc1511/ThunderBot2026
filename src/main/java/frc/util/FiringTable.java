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
        // TODO: These need to be LOCKED IN 100%
        firingTable.add(new FiringDataPoint(0.5, 1800, 0, 1));
        firingTable.add(new FiringDataPoint(1.0, 2000, 0.2, 1.4));
        firingTable.add(new FiringDataPoint(2.0, 2500, 0.2, 1.6));
    }

    // TODO: For now
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
