package frc.robot.subsystems.Cannon;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Helpers;
import frc.util.Constants.Hood;
import frc.util.Constants.Status;
import frc.util.Thunder.Modifiable;
import frc.util.Thunder.ThunderSubsystem;

public class HoodSubsystem extends ThunderSubsystem {
    private TalonFX m_motor;
    private CANcoder m_encoder;
    private DigitalInput m_beamBreakZero;

    private boolean isUsingInbuiltEncoder = false;

    public HoodSubsystem() {
        new Modifiable("isConfirmedZeroed", this, () -> Boolean.FALSE);
        
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration(); 
        hoodConfig.Slot0 = new Slot0Configs()
            .withKP(Constants.Hood.HoodPID.kP).withKI(Constants.Hood.HoodPID.kI).withKD(Constants.Hood.HoodPID.kD);
        hoodConfig.Feedback.RotorToSensorRatio = Constants.Hood.kGearing;
        hoodConfig.Feedback.SensorToMechanismRatio = 1;
        hoodConfig.CurrentLimits.StatorCurrentLimit = Constants.Hood.kStatorCurrentLimit;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        if (!Broken.hoodDisabled) {
            m_encoder = new CANcoder(Constants.IOMap.Hood.kCANcoder);
            m_encoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(Constants.Hood.kCANcoderOffset));
            
            hoodConfig.Feedback.withFusedCANcoder(m_encoder);
            
            m_motor = new TalonFX(Constants.IOMap.Hood.kHoodMotor);
            m_motor.getConfigurator().apply(hoodConfig);

            m_beamBreakZero = new DigitalInput(Constants.IOMap.Hood.kDIObeamBreak);
            if (!Broken.hoodBeamBreakDisabled && isAtZero()) {
                zeroEncodersLightly();
            } else if (Broken.hoodBeamBreakDisabled) {
                zeroEncodersLightly();
            }
            new Trigger(this::isAtZero).onTrue(new InstantCommand(() -> {
                zeroEncodersLightly();
            }));

            if (!Helpers.onCANChain(m_encoder)) {
                zeroEncodersLightly();
            }
        } else {
            m_motor = null;
        }
    }

    @Override
    public void periodic() {
        if (Broken.hoodDisabled) return;
        
        SmartDashboard.putNumber("hood_cancoder_rots", m_encoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("hood_motor_rots", m_motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("hood_vel", m_encoder.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("hood_atPos", atPosition());
        SmartDashboard.putNumber("hood_output_V", m_motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putBoolean("hood_isZeroed", isZeroed());
        SmartDashboard.putBoolean("hood_zeroSensorTripped", isAtZero());
        SmartDashboard.putNumber("hood_err", m_motor.getClosedLoopError().getValueAsDouble());

        if (m_motor.getPosition().getValueAsDouble() < Constants.Hood.Position.BOTTOM.get() || m_motor.getPosition().getValueAsDouble() > Constants.Hood.Position.TOP.get()) {
            halt();
        }

        if (!isZeroed()) {
            CommandScheduler.getInstance().schedule(zero());
        }

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
    }

    private void zeroEncodersLightly() {
        Modifiable isConfirmedZeroed = getField("isConfirmedZeroed");
        if (isConfirmedZeroed != null && isConfirmedZeroed.getValue() instanceof Boolean) isConfirmedZeroed.withValue(() -> Boolean.TRUE);

        double currentPosition = m_encoder.getPosition().getValueAsDouble();
        double newPos = currentPosition - Math.floor(currentPosition);
        if (newPos >= 0.9) newPos -= 1;
        m_encoder.setPosition(newPos); // Remove the full digit; i.e 1.2489 -> 0.2489
    }

    public boolean isAtZero() {
        if (Broken.hoodBeamBreakDisabled) return false;
        return m_beamBreakZero.get();
    }

    public boolean isZeroed() {
        Modifiable isConfirmedZeroed = getField("isConfirmedZeroed");
        if (isConfirmedZeroed != null && isConfirmedZeroed.getValue() instanceof Boolean) return (Boolean) isConfirmedZeroed.getValue();
        return false;
    }

    public boolean atPosition() {
        if (Broken.hoodDisabled) return true;

        return Math.abs(m_motor.getClosedLoopError().getValueAsDouble()) < Constants.Hood.kHoodTolerance && Math.abs(m_motor.getClosedLoopOutput().getValueAsDouble()) < Constants.Hood.kHoodSetpointMaxVelocity;
    }

    public Command zero() {
        if (Broken.hoodDisabled) return Commands.none();
        if (Broken.hoodBeamBreakDisabled) {
            Modifiable isConfirmedZeroed = getField("isConfirmedZeroed");
            if (isConfirmedZeroed != null && isConfirmedZeroed.getValue() instanceof Boolean) isConfirmedZeroed.withValue(() -> true);

            return Commands.none();
        }

        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(-Hood.kZeroingSpeed))
            .isFinished(this::isZeroed)
            .onEnd(() -> m_motor.stopMotor());
    }

    public Command toPosition(Supplier<Double> targetPosition) {
        if (Broken.hoodDisabled) return Commands.none();
        
        return new CommandBuilder(this) 
            .onExecute(() -> m_motor.setControl(new PositionVoltage(targetPosition.get())))
            .isFinished(this::atPosition)
            .onlyIf(this::isZeroed);
    }

    public Command manual_hood(DoubleSupplier speed) {
        if (Broken.hoodDisabled) return Commands.none();
        
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.set(speed.getAsDouble());
            });
    }

    public boolean safeForTrench() {
        if (Broken.hoodDisabled) return false;
        
        return (m_motor.getClosedLoopReference().getValueAsDouble() == Constants.Hood.Position.BOTTOM.get() && atPosition()) || isAtZero();
    }

    public Command stowForTrench() {
        if (Broken.hoodDisabled) return Commands.none();
        
        double currentSetpoint = m_motor.getClosedLoopReference().getValueAsDouble();

        return new CommandBuilder(this) 
            .onExecute(() -> m_motor.setControl(new PositionVoltage(Constants.Hood.Position.TRENCH.get())))
            .onEnd(() -> m_motor.setControl(new PositionVoltage(currentSetpoint)))
            .onlyIf(this::isZeroed);
    }

    public Command forceZeroEncoders() {
        if (Broken.hoodDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> {
                m_encoder.setPosition(0);
                m_motor.setPosition(0);
            });
    }

    public Command halt() {
        if (Broken.hoodDisabled) return new InstantCommand(()->{}, this);

        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.stopMotor();
            });
    }

    public Command setBrakeMode(BooleanSupplier brakeOn) {
        if (Broken.hoodDisabled) return Commands.none();

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
}