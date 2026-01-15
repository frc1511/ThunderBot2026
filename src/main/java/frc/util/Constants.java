package frc.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public class Constants {
    public class IOMap {
        /*
        *    +======================================+
        *    |                                      |
        *    |                                      |
        *    |   _____ _____  ___  ___ ___ ______   |
        *    |  |_   _|  _  | |  \/  |/ _ \| ___ \  |
        *    |    | | | | | | | .  . / /_\ | |_/ /  |
        *    |    | | | | | | | |\/| |  _  |  __/   |
        *    |   _| |_\ \_/ / | |  | | | | | |      |
        *    |   \___/ \___/  \_|  |_\_| |_\_|      |
        *    |                                      |
        *    |                                      |
        *    +======================================+
        */

        public class Swerve {
            public static final int kFLEncoder = 1;
            public static final int kFLDrive = 2;
            public static final int kFLSteer = 3;

            public static final int kFREncoder = 4;
            public static final int kFRDrive = 5;
            public static final int kFRSteer = 6;

            public static final int kBLEncoder = 7;
            public static final int kBLDrive = 8;
            public static final int kBLSteer = 9;

            public static final int kBREncoder = 10;
            public static final int kBRDrive = 11;
            public static final int kBRSteer = 12;

            public static final int kPigeon = 13;
        }
    }

    public class SwerveConstants {
        // Steer PID
        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(20).withKI(0).withKD(0.01)
            .withKS(0).withKV(0).withKA(0) // 2.66
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // Drive PID
        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.01).withKI(0).withKD(0)
            .withKS(0).withKV(0.124);

        private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder; // FusedCANcoder (mfw no pro)

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final Current kSlipCurrent = Amps.of(120); // TODO

        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
            .withClosedLoopRamps(
                new ClosedLoopRampsConfigs()
                    .withVoltageClosedLoopRampPeriod(0.5)
            );
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
            ).withClosedLoopRamps(
                new ClosedLoopRampsConfigs()
                    .withVoltageClosedLoopRampPeriod(0.1)
            );
        private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;

        public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");

        // Max Speed
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.04);

        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.122448979591837;
        private static final double kSteerGearRatio = 21.428571428571427;
        private static final Distance kWheelRadius = Inches.of(2);

        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        // These are only used for simulation
        private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
        // Simulated voltage necessary to overcome friction
        private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);
        
        public static final double kMaxSpeed = 1.0 * kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        public static final SwerveDrivetrainConstants kDrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(kCANBus.getName())
                .withPigeon2Id(IOMap.Swerve.kPigeon)
                .withPigeon2Configs(pigeonConfigs);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);

        // Front Left
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.36962890625);
        private static final boolean kFrontLeftSteerMotorInverted = true;
        private static final boolean kFrontLeftEncoderInverted = false;
        private static final Distance kFrontLeftXPos = Inches.of(15.5);
        private static final Distance kFrontLeftYPos = Inches.of(14);

        // Front Right
        private static final Angle kFrontRightEncoderOffset = Rotations.of(0.197998046875);
        private static final boolean kFrontRightSteerMotorInverted = true;
        private static final boolean kFrontRightEncoderInverted = false;
        private static final Distance kFrontRightXPos = Inches.of(15.5);
        private static final Distance kFrontRightYPos = Inches.of(-14);

        // Back Left
        private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.140625);
        private static final boolean kBackLeftSteerMotorInverted = true;
        private static final boolean kBackLeftEncoderInverted = false;
        private static final Distance kBackLeftXPos = Inches.of(-15.5);
        private static final Distance kBackLeftYPos = Inches.of(14);

        // Back Right
        private static final Angle kBackRightEncoderOffset = Rotations.of(-0.181640625);
        private static final boolean kBackRightSteerMotorInverted = true;
        private static final boolean kBackRightEncoderInverted = false;
        private static final Distance kBackRightXPos = Inches.of(-15.5);
        private static final Distance kBackRightYPos = Inches.of(-14);

        // Module Creation
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
            ConstantCreator.createModuleConstants(
                IOMap.Swerve.kFLSteer, IOMap.Swerve.kFLDrive, IOMap.Swerve.kFLEncoder, kFrontLeftEncoderOffset,
                kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
            ConstantCreator.createModuleConstants(
                IOMap.Swerve.kFRSteer, IOMap.Swerve.kFRDrive, IOMap.Swerve.kFREncoder, kFrontRightEncoderOffset,
                kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
            ConstantCreator.createModuleConstants(
                IOMap.Swerve.kBLSteer, IOMap.Swerve.kBLDrive, IOMap.Swerve.kBLEncoder, kBackLeftEncoderOffset,
                kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
            ConstantCreator.createModuleConstants(
                IOMap.Swerve.kBRSteer, IOMap.Swerve.kBRDrive, IOMap.Swerve.kBREncoder, kBackRightEncoderOffset,
                kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
            );
    }
}
