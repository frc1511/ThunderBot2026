package frc.util;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Helpers {
    public static boolean onCANChain(TalonFX motor) {
        return motor.isConnected(Constants.kCANChainDisconnectTimeout);
    }
    
    public static boolean onCANChain(SparkMax motor) {
        return motor.getLastError() != REVLibError.kCANDisconnected;
    }
    
    public static boolean onCANChain(CANcoder encoder) {
        return encoder.isConnected(Constants.kCANChainDisconnectTimeout);
    }

    public static boolean isRunning(TalonFX motor) {
        return Math.abs(motor.get()) > 0.1;
    }

    public static boolean isRunning(SparkMax motor) {
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
}
