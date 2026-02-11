package frc.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
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

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public class Constants {
    public static boolean kUseSignalLogger = false;

    public static final double kCANChainDisconnectTimeout = 0.4; // in seconds

    public static final double kAntiSpamAlertTimeout = 5; // in seconds

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
        public class Hang {
            public static final int hangMotor = 2;
            public static final int lowerLimit = 0;
            public static final int upperLimit = 1;
        }
      
        public class Intake {
            public static final int pivotMotor = 24;
            public static final int chompMotor = 25;
        }
      
        public class Swerve {
            public static final int kFLEncoder = 7;
            public static final int kFLDrive = 8;
            public static final int kFLSteer = 9;

            public static final int kFREncoder = 1;
            public static final int kFRDrive = 2;
            public static final int kFRSteer = 3;

            public static final int kBLEncoder = 10;
            public static final int kBLDrive = 11;
            public static final int kBLSteer = 12;

            public static final int kBREncoder = 4;
            public static final int kBRDrive = 5;
            public static final int kBRSteer = 6;

            public static final int kPigeon = 13;
        }
      
        public interface Shooter {
            //TODO: actual can ids
            int kShooterMotorA = 21;
            int kShooterMotorB = 22;
            int kHoodMotor = 23;
            int kturretMotor = 24;
        }

        public interface Storage {
            int kSpindexerMotor = 41;
            int kKickerMotor = 42;
        }
    }

    protected interface BasePID {
        double kP = 0.0d;
        double kI = 0.0d;
        double kD = 0.0d;
    }

    public interface BlinkyBlinky {
        int kPort = 0;

        int kLength = 40;

        enum Mode {
            NONE, INTAKING, HUNG, FIRE_READY, HOME, TRENCH_SAFE
        }
    }

    public interface Storage {
        interface Spindexer {
           double kSpeed = 1;

            /** Seconds */
            enum Duration {
                FULL_BAY(4), PARTIAL_BAY(2), AUTO_BAY(1), INTAKE(1.0E9);

                private double m_value;
                private Duration(double value) {
                    this.m_value = value;
                }

                public double get() {
                    return this.m_value;
                }
            }
        }

        interface Kicker {
            double kSpeed = 1.0;
        }
    }
    
    public interface Shooter {
        double kMaxShooterSpeed = 0.5d;
        double kShooterAtSpeedTolerance = 0.1d;
        double kTurretTolerance = 0.5d;
        double kHoodTolerance = 0.5d;

        public interface TurretPID extends BasePID {}
        public interface HoodPID extends BasePID {}

        double kTargetShooterRPM = 3000;

        public interface ShooterPID extends BasePID {
            double kP = 0.2;
            double kI = .6;
            double kD = 0.01;
        }
    }

    public class Hunger {
        public interface Intake {
            double kEatSpeed = 0.3;

            String intakeCommandName = "intake";
        }

        public interface Pivot {
            interface PivotPID extends BasePID { }

            enum Position {
                TOP(31), BOTTOM(40); // TODO: these aren't real

                private double m_value;

                private Position(double value) {
                    this.m_value = value;
                }

                public double get() {
                    return this.m_value;
                }
            }

            double kTolerance = 0.5d;
        }
    }

    public interface Cannon {
        interface Hood {
            double kBottomPosition = 0;
        }
    }
 
    public interface HangConstants {
        double kSetpointMaxVelocity = 0.02; // Prevents flybys
        double kSetpointPositionTolerance = 0.05;
        double kZeroingSpeed = -0.1;
        double kMaxDeploySpeed = 0.4; // TODO: DOUBLE FOR LATER TESTING // Extending Hanger
        double kMaxPullSpeed = -0.3;  // TODO: DOUBLE FOR LATER TESTING // Retracting Hanger (pulling the robot up on the bar)
        double kMaxDeployDistanceRotations = 33;
        double kMaxPullDistanceRotations = 8; // This is not a delta and is absolute to the zero, DON'T confuse it for how much the robot is pulling down
    }
  
    public class Swerve {
        // Max Speed
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.76); // DON"T TOUCH, USE MULTIPLIER FOR MAX SPEED INSTEAD

        private static final double kCoupleRatio = 3.125;

        private static final double kDriveGearRatio = 5.357142857142857;
        private static final double kSteerGearRatio = 18.75;
        private static final Distance kWheelRadius = Inches.of(2);

        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        public static final double kGotoXYTolerance = 0.1d;
        public static final double kGotoThetaTolerance = 3d / 180d * Math.PI;

        // These are only used for simulation
        private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
        // Simulated voltage necessary to overcome friction
        private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        public static final double kMaxSpeed = 0.25 * kSpeedAt12Volts.in(MetersPerSecond); // % Multiplier | kSpeedAt12Volts desired top speed
        public static final double kMaxAngularRate = .5 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        public static final double kMaxAngularAcceleration = .5 * RotationsPerSecondPerSecond.of(.75).in(RadiansPerSecondPerSecond);

        public static final double kVelocityDeadband = 0.40 * kMaxSpeed; // % Multiplier 
        public static final double kAngularVelocityDeadband = 0.10 * kMaxAngularRate; // % Multiplier

        private interface XYPID extends BasePID {
            double kP = 3;
            double kI = .1;
        }
        private interface ThetaPID extends BasePID {
            double kP = 6;
            double kD = 0.101;
        }
        public static final PIDController kHolonomicXPIDController = new PIDController(XYPID.kP, XYPID.kI, XYPID.kD);
        public static final PIDController kHolonomicYPIDController = new PIDController(XYPID.kP, XYPID.kI, XYPID.kD);
        public static final ProfiledPIDController kHolonomicThetaPIDController = new ProfiledPIDController(ThetaPID.kP, ThetaPID.kI, ThetaPID.kD, new Constraints(kMaxAngularRate, kMaxAngularAcceleration));

        public static Pose2d targetPose = new Pose2d(
            11.887319 - 1,
            7.41196,
            new Rotation2d(-Math.PI / 2.0d)
        );

        // These are welded
        public static Pose2d redHubCenterPose = new Pose2d(
            11.887319,
            4.034631,
            new Rotation2d()
        );
        public static Pose2d blueHubCenterPose = new Pose2d(
            4.5998835,
            4.034631,
            new Rotation2d()
        );

        // Steer PID
        public static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(38.568).withKI(0).withKD(1.7601)
            .withKS(0.080138).withKV(2.1689).withKA(0.060677) // 2.66
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // Drive PID
        public static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.16461).withKI(0).withKD(0)
            .withKS(0.077536).withKV(0.11232); // 0.11232 ; 0.0071659

        private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder; // FusedCANcoder (mfw no pro)

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final Current kSlipCurrent = Amps.of(120); // TODO

        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
            // .withClosedLoopRamps(
            //     new ClosedLoopRampsConfigs()
            //         .withVoltageClosedLoopRampPeriod(0.1)
            // );
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
            ).withClosedLoopRamps(
                new ClosedLoopRampsConfigs()
                    .withVoltageClosedLoopRampPeriod(0.2));
        private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;

        public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");

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
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.38720703125);
        private static final boolean kFrontLeftSteerMotorInverted = true;
        private static final boolean kFrontLeftEncoderInverted = false;
        private static final Distance kFrontLeftXPos = Inches.of(9.75);
        private static final Distance kFrontLeftYPos = Inches.of(12.25);

        // Front Right
        private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.1201171875);
        private static final boolean kFrontRightSteerMotorInverted = true;
        private static final boolean kFrontRightEncoderInverted = false;
        private static final Distance kFrontRightXPos = Inches.of(9.75);
        private static final Distance kFrontRightYPos = Inches.of(-12.25);

        // Back Left
        private static final Angle kBackLeftEncoderOffset = Rotations.of(0.071533203125);
        private static final boolean kBackLeftSteerMotorInverted = true;
        private static final boolean kBackLeftEncoderInverted = false;
        private static final Distance kBackLeftXPos = Inches.of(-9.75);
        private static final Distance kBackLeftYPos = Inches.of(12.25);

        // Back Right
        private static final Angle kBackRightEncoderOffset = Rotations.of(-0.055419921875);
        private static final boolean kBackRightSteerMotorInverted = true;
        private static final boolean kBackRightEncoderInverted = false;
        private static final Distance kBackRightXPos = Inches.of(-9.75);
        private static final Distance kBackRightYPos = Inches.of(-12.25);

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
