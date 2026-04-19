package frc.robot.subsystems.Cannon;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Helpers;
import frc.util.Constants.Hood;
import frc.util.Constants.Status;
import frc.util.Thunder.ThunderSubsystem;

public class HoodSubsystem extends ThunderSubsystem {
    private TalonFX m_motor;
    private CANcoder m_encoder;
    private DigitalInput m_beamBreakZero;

    private boolean isUsingInbuiltEncoder = false;

    private double trueSetpoint = Constants.Hood.Position.BOTTOM.get();

    private DoubleSupplier optimalAngleSupplier = () -> 0;

    private BooleanSupplier isReadyToMove = () -> isZeroed() || Helpers.isBypassModeEnabled();

    public HoodSubsystem() {
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0 = new Slot0Configs()
            .withKP(Constants.Hood.HoodPID.kP).withKI(Constants.Hood.HoodPID.kI).withKD(Constants.Hood.HoodPID.kD)
            .withKS(Constants.Hood.HoodPID.kS).withKV(Constants.Hood.HoodPID.kV).withKA(Constants.Hood.HoodPID.kA);
        hoodConfig.Feedback.RotorToSensorRatio = Constants.Hood.kGearing;
        hoodConfig.Feedback.SensorToMechanismRatio = 1;
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Hood.HoodMotionMagic.kVel;
        hoodConfig.MotionMagic.MotionMagicAcceleration = Constants.Hood.HoodMotionMagic.kAccel;
        hoodConfig.MotionMagic.MotionMagicJerk = Constants.Hood.HoodMotionMagic.kJerk;
        hoodConfig.CurrentLimits.StatorCurrentLimit = Constants.Hood.kStatorCurrentLimit;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true)
            );

        if (!Broken.hoodDisabled) {
            m_encoder = new CANcoder(Constants.IOMap.Hood.kCANcoder);
            m_encoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(Constants.Hood.kCANcoderOffset));
            
            hoodConfig.Feedback.withFusedCANcoder(m_encoder);
            
            m_motor = new TalonFX(Constants.IOMap.Hood.kHoodMotor);
            m_motor.getConfigurator().apply(hoodConfig);

            m_motor.getClosedLoopReference().setUpdateFrequency(100);
            m_motor.getClosedLoopReferenceSlope().setUpdateFrequency(200);

            m_beamBreakZero = new DigitalInput(Constants.IOMap.Hood.kDIObeamBreak);
            if (!Broken.hoodBeamBreakDisabled && isAtZero() ||
                    Broken.hoodBeamBreakDisabled ||
                    !Helpers.onCANChain(m_encoder) ||
                    Helpers.isBypassModeEnabled()) {
                forceZeroEncoders();
            }

            new Trigger(() -> Helpers.onCANChain(m_encoder)).onTrue(new InstantCommand(() -> {
                if (!Helpers.onCANChain(m_encoder)) {
                    m_motor.getConfigurator().apply(new FeedbackConfigs()
                        .withRotorToSensorRatio(1)
                        .withSensorToMechanismRatio(Constants.Hood.kGearing));
                    isUsingInbuiltEncoder = true;
                } else if (isUsingInbuiltEncoder && Helpers.onCANChain(m_encoder)) {
                    m_motor.getConfigurator().apply(new FeedbackConfigs()
                        .withFusedCANcoder(m_encoder)
                        .withRotorToSensorRatio(Constants.Hood.kGearing)
                        .withSensorToMechanismRatio(1));
                    isUsingInbuiltEncoder = false;
                }
            }));
        } else {
            m_motor = null;
        }
    }

    @Override
    public void periodic() {
        if (Broken.hoodDisabled) return;
        
        SmartDashboard.putNumber("Hood / Cancoder ROT", m_encoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood / Motor ROT", m_motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood / Vel RPS", m_encoder.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Hood / At Target Position", atPosition());
        SmartDashboard.putNumber("Hood / Motor Output V", m_motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Hood / Final Setpoint", trueSetpoint);
        // SmartDashboard.putNumber("Hood / profiled_setpoint", m_motor.getClosedLoopReference().getValueAsDouble());
        // SmartDashboard.putNumber("Hood / profiled_setpoint_d-dx", m_motor.getClosedLoopReferenceSlope().getValueAsDouble());
        SmartDashboard.putBoolean("Hood / Is Zeroed", isZeroed());
        SmartDashboard.putBoolean("Hood / Zero Sensor Tripped", isAtZero());
        SmartDashboard.putNumber("Hood / Setpoint Error", m_motor.getClosedLoopError().getValueAsDouble());

        SmartDashboard.putNumber("SOTM / Hood Target Theta", optimalAngleSupplier.getAsDouble());
    }

    @Override
    public void hddlPeriodic() {
        SmartDashboard.putNumber("Hood / Profiled Setpoint", m_motor.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("Hood / Profiled Setpoint Slope", m_motor.getClosedLoopReferenceSlope().getValueAsDouble());
    }

    public Command setCoastMode() {
        if (Broken.pivotDisabled) return Commands.none(); 
        return new CommandBuilder()
            .onInitialize(() -> {
                m_motor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
            })
            .onEnd(() -> {
                m_motor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
            });
    }

    public boolean isAtZero() {
        if (Broken.hoodBeamBreakDisabled) return false;
        return m_beamBreakZero.get();
    }

    private boolean isConfirmedZeroed = false;
    public boolean isZeroed() {
        return isConfirmedZeroed;
    }

    public boolean atPosition() {
        if (Broken.hoodDisabled || Helpers.isBypassModeEnabled()) return true;

        return Math.abs(trueSetpoint - m_motor.getPosition().getValueAsDouble()) < Constants.Hood.kHoodTolerance && Math.abs(m_encoder.getVelocity().getValueAsDouble()) < Constants.Hood.kHoodSetpointMaxVelocity;
    }

    public Command zero() {
        if (Broken.hoodDisabled) return CommandBuilder.none(this);
        if (Broken.hoodBeamBreakDisabled) {
            forceZeroEncoders();
            isConfirmedZeroed = true;
            return CommandBuilder.none(this);
        }

        return new CommandBuilder(this)
            .onExecute(() -> {
                if (Helpers.isBypassModeEnabled()) { // Don't run zeroing procedures if in pit mode plus
                    m_motor.stopMotor();
                } else {
                    if (!isConfirmedZeroed) {
                        m_motor.set(-Hood.kZeroingSpeed);
                        if (isAtZero()) {
                            forceZeroEncoders();
                            m_motor.stopMotor();
                        }
                    } else {
                        m_motor.stopMotor();
                    }
                }
            });
    }

    public Command toPosition(Supplier<Double> targetPosition) {
        if (Broken.hoodDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this) 
            .onInitialize(() -> {
                trueSetpoint = targetPosition.get();
            })
            .onExecute(() -> {
                trueSetpoint = targetPosition.get();
                m_motor.setControl(new MotionMagicVoltage(trueSetpoint));
            })
            .isFinished(this::atPosition)
            .onlyIf(isReadyToMove);
    }

    @Override
    public Command manual(DoubleSupplier speed) {
        if (Broken.hoodDisabled) return CommandBuilder.none(this);
        
        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(speed.getAsDouble()))
            .onEnd(() -> m_motor.stopMotor());
    }

    public boolean safeForTrench() {
        if (Broken.hoodDisabled) return false;
        
        return (trueSetpoint == Constants.Hood.Position.BOTTOM.get() && atPosition()) || isAtZero();
    }

    public Command stowForTrench() {
        if (Broken.hoodDisabled) return CommandBuilder.none(this);
        
        double currentSetpoint = m_motor.getClosedLoopReference().getValueAsDouble();

        return new CommandBuilder(this) 
            .onInitialize(() -> {
                trueSetpoint = Constants.Hood.Position.TRENCH.get();
            })
            .onExecute(() -> {
                trueSetpoint = Constants.Hood.Position.TRENCH.get();
                m_motor.setControl(new MotionMagicVoltage(trueSetpoint));
            })
            .onEnd(() -> m_motor.setControl(new MotionMagicVoltage(currentSetpoint)))
            .onlyIf(isReadyToMove);
    }

    public void forceZeroEncoders() {
        if (Broken.hoodDisabled) return;

        isConfirmedZeroed = true;
        m_encoder.setPosition(0);
        m_motor.setPosition(0);
    }

    public Command halt() {
        if (Broken.hoodDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.stopMotor();
            });
    }

    public Command setBrakeMode(BooleanSupplier brakeOn) {
        if (Broken.hoodDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(brakeOn.getAsBoolean() ? NeutralModeValue.Brake : NeutralModeValue.Coast));
            });
    }

    public Status status() {
        if (Broken.hoodDisabled) return Status.DISABLED;
        if (!m_motor.isConnected(Constants.kCANChainDisconnectTimeout)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_motor)) return Status.ACTIVE;
        return Status.IDLE;
    }

    public void setOptimalAngleGetter(DoubleSupplier supplier) {
        optimalAngleSupplier = supplier;
    }

    public Command toOptimalPosition() {
        if (Broken.hoodDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onInitialize(() -> {
                trueSetpoint = optimalAngleSupplier.getAsDouble();
            })
            .onExecute(() -> {
                trueSetpoint = optimalAngleSupplier.getAsDouble();
                m_motor.setControl(new MotionMagicVoltage(trueSetpoint));
            })
            .isFinished(this::atPosition)
            .onlyIf(isReadyToMove);
    }

    public Constants.Hood.Position getTargetPosition() {
        for (Constants.Hood.Position preset : Constants.Hood.Position.getAll()) {
            if (trueSetpoint == preset.get()) {
                return preset;
            }
        }
        return Constants.Hood.Position.BOTTOM;
    }
}