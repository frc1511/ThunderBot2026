package frc.util;

import java.util.HashSet;
import java.util.Optional;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Helpers {
    public static boolean alertsShouldUseConsole = false;

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

    public static boolean isHubActive(boolean withOffset) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime() - (withOffset ? Constants.kTimerAheadTime_sec : 0);
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
            // If we have invalid game data, assume hub is active.
            return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    public static Trigger hubToBecomeActive() {
        return new Trigger(() -> isHubActive(true));
    }

    private static boolean pitModePlusEnabled = false;
    public static void setPitModePlus(boolean enabled) {
        pitModePlusEnabled = enabled;
    }
    /**
     * Pitmode+ is a stepping stone up from pit mode that ~~removes a bunch of safety features.~~ allows you to bypass some requirements so that you can easily test things like the shooter without having to worry if the hood has reached its goal yet. The caveat is that the robot just assumes you set things up correctly as it skips checking so you can test faster, meaning you should probably read the functionality of whatever you're testing when in pitmode+ before testing it. 
     */
    public static boolean isPitModePlusEnabled() {
        return pitModePlusEnabled;
    }

    private static boolean bypassModeEnabled = false;
    public static void setBypassMode(boolean enabled) {
        bypassModeEnabled = enabled;
    }
    /**
     * Bypass mode is a stepping stone up from pit mode that ~~removes a bunch of safety features.~~ allows you to bypass some requirements so that you can easily test things like the shooter without having to worry if the hood has reached its goal yet. The caveat is that the robot just assumes you set things up correctly as it skips checking so you can test faster, meaning you should probably read the functionality of whatever you're testing when in pitmode+ before testing it. 
     */
    public static boolean isBypassModeEnabled() {
        return bypassModeEnabled;
    }

    public static Pose2d getTargetHangPose(Pose2d currentPose) {
        double xOffset = Constants.HangConstants.kHangCenterDisplacementX;
        double yOffset = Constants.HangConstants.kHangCenterDisplacementY;
        Pose2d target = Pose2d.kZero;
        double y = currentPose.getY();
        if (isBlueAlliance()) {
            if (y <= 4d) {
                target = new Pose2d(1.0375 + xOffset, 3.2523 - yOffset, Rotation2d.kZero);
            } else {
                target = new Pose2d(1.0375 - xOffset, 4.2391 + yOffset, Rotation2d.kPi);
            }
        } else {
            if (y <= 4d) {
                target = new Pose2d(15.4325 + -.05 + xOffset, 3.7301 - yOffset, Rotation2d.kZero);
            } else {
                target = new Pose2d(15.4325 + -.05 - xOffset, 4.9169 + yOffset, Rotation2d.kPi);
            }
        }

        return target;
    }
}