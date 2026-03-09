package frc.util;

import java.util.HashSet;
import java.util.Optional;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Helpers {
    public static boolean onCANChain(TalonFX motor) {
        if (motor == null) return false;
        return motor.isConnected(Constants.kCANChainDisconnectTimeout);
    }
    
    public static boolean onCANChain(SparkMax motor) {
        if (motor == null) return false;
        return motor.getLastError() != REVLibError.kCANDisconnected;
    }
    
    public static boolean onCANChain(CANcoder encoder) {
        if (encoder == null) return false;
        return encoder.isConnected(Constants.kCANChainDisconnectTimeout);
    }

    public static boolean isRunning(TalonFX motor) {
        if (motor == null) return false;
        return Math.abs(motor.get()) > 0.1;
    }

    public static boolean isRunning(SparkMax motor) {
        if (motor == null) return false;
        return Math.abs(motor.get()) > 0.1;
    }
    
    public static Pose2d allianceHub() {
        Alliance alliance;
        if (DriverStation.getAlliance().isEmpty()) {
            alliance = DriverStation.Alliance.Blue;
        } else {
            alliance = DriverStation.getAlliance().get();
        }

        if (alliance == DriverStation.Alliance.Blue) {
            return Constants.Swerve.blueHubCenterPose;
        }
        return Constants.Swerve.redHubCenterPose;
    }

    public static double RPMtoRPS(double RPM) {
        return RPM / 60;
    }

    public static double RPStoRPM(double RPS) {
        return RPS * 60;
    }

    public static double lerp(double v0, double v1, double t) {
        return v0 + t * (v1 - v0);
    }

    /**
     * {@code wantedTarget} is the target you want to be moving towards.
     * {@code realTarget} is the target the system thinks it's moving towards (often a method from a pid controller).
     */
    public static boolean ensureTarget(double wantedTarget, double realTarget, double tolerance) {
        return Math.abs(realTarget - wantedTarget) < tolerance;
    }

    public static int clamp(int x, int lo, int hi) {
        return Math.min(Math.max(x, lo), hi);
    }

    public static double clamp(double x, double lo, double hi) {
        return Math.min(Math.max(x, lo), hi);
    }

    // Will return true if the average distance to the average value is less than the tolerance
    public static boolean standardDeviation(HashSet<Double> values, double tolerance) {
        double total = values.stream().mapToDouble(Double::doubleValue).sum();
        double average = total / values.size();

        double totalDistance = values.stream().mapToDouble(value -> average - value).sum();
        double averageDistance = totalDistance / values.size();

        return Math.abs(averageDistance) < tolerance;
    }

    public static boolean isBlueAlliance() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
        }
        return false;
    }
}
