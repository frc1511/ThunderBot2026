package frc.util;

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