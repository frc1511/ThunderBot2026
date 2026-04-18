package frc.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;

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
    public static boolean kUseDataLog = true;
    public static boolean kUseHDDL = false; // High density data logging
    public static boolean kLogTimeAndJoystick = false;
    public static boolean kLogSafetyWatchdogStatuses = false;
    public static double kHDDLRate = 200d; // Hz

    public static final double kCANChainDisconnectTimeout = 0.5; // In seconds

    public static final double kAntiSpamAlertTimeout = 5; // In seconds

    public static final double kControllerDeadzone = 0.3;

    public static final boolean kDisconecctOnCANBroken = false;

    public static final int kTimerAheadTime_sec = 2;

    public static enum Status {
        IDLE,
        ACTIVE,
        DISABLED,
        DISCONNECTED
    }

    public static enum SysIDMechanism {
        NONE,
        DRIVE,
        SHOOTER
    }
    public static final SysIDMechanism kSysIDMode = SysIDMechanism.NONE;

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
            public static final int kHangMotor = 14;
            public static final int kDIOlowerLimit = 0;
            public static final int kDIOupperLimit = 1;
            public static final int kAnalogDistance = 0;
        }
      
        public class Intake {
            public static final int kCANcoder = 15;
            public static final int kPivotMotor = 16;
            public static final int kChompMotorRight = 17;
            public static final int kChompMotorLeft = 41;
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
      
        public class Shooter {
            public static final int kShooterMotorA = 21; // Right
            public static final int kShooterMotorB = 20; // Left
            public static final int kTurretMotor = 40;
        }
        
        public class Hood {
            public static final int kHoodMotor = 23;
            public static final int kCANcoder = 22;
            public static final int kDIObeamBreak = 2;
        }

        public class Storage {
            public static final int kSpindexerMotor = 18;
            public static final int kKickerMotor = 19;
        }

        public class BlinkyBlinky {
            public static final int kPWMport = 9;
        }
    }

    protected interface BasePID {
        double kP = 0.0d;
        double kI = 0.0d;
        double kD = 0.0d;
        double kS = 0.0d;
        double kV = 0.0d;
        double kA = 0.0d;
        double kG = 0.0d;
        double kCos = 0.0d;
    }

    protected interface ProfiledPID {
        double kVel = 0;
        double kAccel = 0;
        double kJerk = 0;
    }

    public interface BlinkyBlinky {
        int kLength = 72;

        enum Mode {
            NONE, INTAKING, HUNG, FIRE_READY, HOME, TRENCH_SAFE, PIT, OFF
        }
    }

    public interface Storage {
        interface Spindexer {
           double kSpeed = 1.0d;

            /** Seconds */
            enum Duration {
                FULL_BAY(4), PARTIAL_BAY(2), AUTO_BAY(1), FOREVER(1.0E9), INTAKE(1.0E9), AGGREGATE(1);

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
            double kTargetKickerRPM = 2000;
            double kRPMTolerance = 350;

            public interface KickerPID extends BasePID {
                double kP = 150;
                double kI = 5;
                double kV = 0.1;
            }

            public interface KickerMotionMagic extends ProfiledPID {
                double kAccel = 120;
                double kJerk = 140;
            }
        }
    }

    public interface Shooter {
        double kMaxShooterSpeed = 0.5d;
        double kShooterAtSpeedTolerance = 100d;
        double kTargetShooterRPM = 1900d;
        double kFeedRPM = 2200d;

        public interface TurretPID extends BasePID {}
        double kTurretTolerance = 0.5d;

        public interface ShooterPID extends BasePID {
            double kP = .1;
            double kI = 0;
            double kD = 0;
            double kS = 0.20022;
            double kV = 0.11647;
            double kA = 0.01009;
        }

        public interface ShooterMotionMagic extends ProfiledPID {
            double kAccel = 120;
            double kJerk = 140;
        }

        double kReverseSpeed = -0.2d;

        int kFuelEstimationLookback = 6;
    }

    public interface Hood {
        double kHoodTolerance = 0.015d;
        double kZeroingSpeed = 0.08d; // Just know that zeroing doesn't need to be precise, just needs to see it within a rotation
        enum Position {
            BOTTOM(0), TOP(2), FEED(1), TRENCH(.1), HUB(1.9);

            private double m_value;

            public double get() {
                return m_value;
            }

            private Position(double value) {
                m_value = value;
            }

            static public Set<Position> getAll() {
                return Set.of(BOTTOM, TOP, FEED, TRENCH, HUB);
            }
        }
        public interface HoodPID extends BasePID {
            double kP = 22;
            double kI = 8;
            double kD = 0;
            double kS = 0;
            double kV = 0.12;
        }

        public interface HoodMotionMagic extends ProfiledPID {
            double kVel = 350;
            double kAccel = 60;
            double kJerk = 250;
        }

        double kStatorCurrentLimit = 40;
        double kHoodSetpointMaxVelocity = 0.015d; // Prevents flybys
        double kCANcoderOffset = 0.931781d;
        double kGearing = 9/1;
    }

    public class Hunger {
        public interface Intake {
            double kEatRPM = 3000;
            double kEatPercent = 1.0;

            interface IntakePID extends BasePID { 
                double kP = 8.0;
                double kI = 0;
                double kD = 0;
            }

            String intakeCommandName = "intake";
        }

        public interface Pivot {
            interface PivotPID extends BasePID { 
                double kP = 6.5;
                double kI = .00;
                double kD = 0.0;
                double kS = .25;
                double kCos = .35;
            }
            double kCosRatio = 1;

            enum Position {
                TOP(0.35), HALFWAY_DOWN(0.233), BOTTOM(0.0); // They change alot but should be approx right

                private double m_value;

                private Position(double value) {
                    this.m_value = value;
                }

                public double get() {
                    return this.m_value;
                }
            }
            double kCANcoderOffset = 0.07870;

            double kTolerance = 0.015d;
            double kBigTolerance = 0.02d;

            double kEncoderConversionFactor = 1d/90d;
        }
    }

    public interface HangConstants {
        public interface HangPID extends BasePID {
            double kP = 0.09;
            double kI = 0;
            double kD = 0;
        }

        double kSetpointMaxVelocity = 0.02; // Prevents flybys
        double kSetpointPositionTolerance = 0.25;
        double kZeroingSpeed = -0.1;
        double kMaxDeploySpeed = 0.9; // Extending Hanger
        double kMaxPullSpeed = -0.9;  // Retracting Hanger (pulling the robot up on the bar)
        double kMaxDeployDistanceRotations = 34;
        double kMaxPullDistanceRotations = 6; // This is NOT a delta and is absolute to the zero, DON'T confuse it for how much the robot is pulling down
        double kJostleAmplitude = 0.5;
        double kTrenchSafeDistanceRotations = 1;

        double kTowerDistanceFromWallX = 1.2065;
        double kHangCenterDisplacementX = 0.12954;
        double kHangCenterDisplacementY = 0.3048;
    }

    public class Swerve {
        public static final double kTimeOfFlightConvergenceTolerance = 0.01;
        public static final int kTimeOfFlightConvergenceMaxRecursions = 20;
        public static final double kMaximumLimelightDistance = 3d;
        public static final double kTemporarySlowdownAmount = .15d;

        // Max Speed
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.9); // DON"T TOUCH, USE MULTIPLIER FOR MAX SPEED INSTEAD

        public static final double kSpeedStep = 0.1; // Amount to step speed for the increase/decrease buttons

        public static final double kShooterOffset = .351; // meters
        public static final double kTrenchLockMaxDist = 3; // meters

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

        public static final double kMaxSpeed = 0.8 * kSpeedAt12Volts.in(MetersPerSecond); // % Multiplier * kSpeedAt12Volts desired top speed
        public static final double kMaxAngularRate = 1d * DegreesPerSecond.of(600).in(RadiansPerSecond);
        public static final double kMaxAngularAcceleration = 1d * DegreesPerSecondPerSecond.of(240).in(RadiansPerSecondPerSecond);

        public static final double kVelocityDeadband = 0.05 * kMaxSpeed; // % Multiplier 
        public static final double kAngularVelocityDeadband = 0.05 * kMaxAngularRate; // % Multiplier

        public interface XYPID extends BasePID {
            double kP = 3;
            double kI = .1;
        }
        public interface ThetaPID extends BasePID {
            double kP = 8;
            double kD = 1;
        }

        public interface ThetaAutoPID extends BasePID {
            double kP = 9.5;
            double kD = 0.5;
        }
        public static final PIDController kHolonomicXPIDController = new PIDController(XYPID.kP, XYPID.kI, XYPID.kD);
        public static final PIDController kHolonomicYPIDController = new PIDController(XYPID.kP, XYPID.kI, XYPID.kD);
        public static final ProfiledPIDController kHolonomicThetaPIDController = new ProfiledPIDController(ThetaPID.kP, ThetaPID.kI, ThetaPID.kD, new Constraints(kMaxAngularRate, kMaxAngularAcceleration));
        static {
            kHolonomicThetaPIDController.setTolerance(.05, .05);
        }

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
            .withKS(0.080138).withKV(2.1689).withKA(0.060677)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // Drive PID
        public static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.16461).withKI(0).withKD(0)
            .withKS(0.077536).withKV(0.11232);

        private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder; // FusedCANcoder (mfw no pro)

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final Current kSlipCurrent = Amps.of(106);

        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(30))
                    .withSupplyCurrentLimitEnable(true)
            );
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true)
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
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.455322265625);
        private static final boolean kFrontLeftSteerMotorInverted = true;
        private static final boolean kFrontLeftEncoderInverted = false;
        private static final Distance kFrontLeftXPos = Inches.of(9.875);
        private static final Distance kFrontLeftYPos = Inches.of(12.5);

        // Front Right
        private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.310791015625);
        private static final boolean kFrontRightSteerMotorInverted = true;
        private static final boolean kFrontRightEncoderInverted = false;
        private static final Distance kFrontRightXPos = Inches.of(9.875);
        private static final Distance kFrontRightYPos = Inches.of(-12.5);

        // Back Left
        private static final Angle kBackLeftEncoderOffset = Rotations.of(0.33349609375);
        private static final boolean kBackLeftSteerMotorInverted = true;
        private static final boolean kBackLeftEncoderInverted = false;
        private static final Distance kBackLeftXPos = Inches.of(-9.875);
        private static final Distance kBackLeftYPos = Inches.of(12.5);

        // Back Right
        private static final Angle kBackRightEncoderOffset = Rotations.of(-.405762);
        private static final boolean kBackRightSteerMotorInverted = true;
        private static final boolean kBackRightEncoderInverted = false;
        private static final Distance kBackRightXPos = Inches.of(-9.875);
        private static final Distance kBackRightYPos = Inches.of(-12.5);


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
