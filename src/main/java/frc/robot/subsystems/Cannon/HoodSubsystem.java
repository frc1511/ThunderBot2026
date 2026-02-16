package frc.robot.subsystems.Cannon;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Alert;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Helpers;
import frc.util.ThunderSubsystem;
import frc.util.Constants.Status;

public class HoodSubsystem extends SubsystemBase implements ThunderSubsystem {
    private TalonFX m_motor;
    private CANcoder m_encoder;

    private boolean isUsingInbuiltEncoder = false;

    public HoodSubsystem() {
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration(); 
        hoodConfig.Slot0 = new Slot0Configs()
            .withKP(Constants.Shooter.HoodPID.kP).withKI(Constants.Shooter.HoodPID.kI).withKD(Constants.Shooter.HoodPID.kD);
        hoodConfig.Feedback.RotorToSensorRatio = 9/1;
        hoodConfig.Feedback.SensorToMechanismRatio = 1;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        if (!Broken.hoodDisabled) {
            m_encoder = new CANcoder(Constants.IOMap.Shooter.kCANCoder);
            m_encoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(0.20849609375+0.105224609375));
            
            hoodConfig.Feedback.withFusedCANcoder(m_encoder);
            
            m_motor = new TalonFX(Constants.IOMap.Shooter.kHoodMotor);
            m_motor.getConfigurator().apply(hoodConfig);
        } else {
            m_motor = null;
        }
    }

    public void periodic() {
        if (Broken.hoodDisabled) return;
        SmartDashboard.putNumber("hood_cancoder_rots", m_encoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("hood_motor_rots", m_motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("hood_vel", m_encoder.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("hood_atPos", atPosition());

        if (m_motor.getPosition().getValueAsDouble() < 0 || m_motor.getPosition().getValueAsDouble() > 2) {
            halt();
        }

        if (!Helpers.onCANChain(m_encoder)) {
            m_motor.getConfigurator().apply(new FeedbackConfigs().withRotorToSensorRatio(1).withSensorToMechanismRatio(9/1));
            isUsingInbuiltEncoder = true;
        } else if (isUsingInbuiltEncoder && Helpers.onCANChain(m_encoder)) {
            m_motor.getConfigurator().apply(new FeedbackConfigs().withFusedCANcoder(m_encoder).withRotorToSensorRatio(9/1).withSensorToMechanismRatio(1));
            isUsingInbuiltEncoder = false;
        }
    }

    public boolean atPosition() {
        if (Broken.hoodDisabled) return true;

        return m_motor.getClosedLoopError().getValueAsDouble() < Constants.Shooter.kHoodTolerance && Math.abs(m_motor.getClosedLoopOutput().getValueAsDouble()) < Constants.Shooter.kHoodSetpointMaxVelocity;
    }

    public Command toPosition(Supplier<Double> targetPosition) {
        if (Broken.hoodDisabled) return Commands.none();
        
        return new CommandBuilder(this) 
            .onExecute(() -> m_motor.setControl(new PositionVoltage(targetPosition.get())))
            .isFinished(this::atPosition)
            .onEnd(this::halt);
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
        
        return m_motor.getClosedLoopReference().getValueAsDouble() == Constants.Cannon.Hood.kBottomPosition && atPosition();
    }

    public Command zero() {
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

    public Status status() {
        if (Broken.hoodDisabled) return Status.DISABLED;
        if (!m_motor.isConnected(Constants.kCANChainDisconnectTimeout)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_motor)) return Status.ACTIVE;
        return Status.IDLE;
    }
}