package frc.robot.subsystems.Cannon;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Helpers;
import frc.util.Constants.Status;

public class HoodSubsystem extends SubsystemBase {
    private TalonFX m_motor;

    public HoodSubsystem() {
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration(); 
        hoodConfig.Slot0 = new Slot0Configs()
            .withKP(Constants.Shooter.HoodPID.kP).withKI(Constants.Shooter.HoodPID.kI).withKD(Constants.Shooter.HoodPID.kD);

        if (!Broken.hoodDisabled) {
            m_motor = new TalonFX(Constants.IOMap.Shooter.kHoodMotor);
            m_motor.getConfigurator().apply(hoodConfig);
        } else {
            m_motor = null;
        }
    }

    public boolean atPosition() {
        if (Broken.hoodDisabled) return true;

        return m_motor.getClosedLoopError().getValueAsDouble() < Constants.Shooter.kHoodTolerance;
    }

    public Command toPosition(Supplier<Double> targetPosition) {
        if (Broken.hoodDisabled) return Commands.none();
        
        return new CommandBuilder(this) 
            .onExecute(() -> m_motor.setControl(new PositionVoltage(targetPosition.get())))
            .isFinished(this::atPosition);
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

    public Status status() {
        if (Broken.hoodDisabled) return Status.DISABLED;
        if (!m_motor.isConnected(Constants.kCANChainDisconnectTimeout)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_motor)) return Status.ACTIVE;
        return Status.IDLE;
    }
}