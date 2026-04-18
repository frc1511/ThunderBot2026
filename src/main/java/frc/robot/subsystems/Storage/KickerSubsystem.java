package frc.robot.subsystems.Storage;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Constants.Status;
import frc.util.Constants.Storage.Kicker.KickerPID;
import frc.util.Thunder.ThunderSubsystem;
import frc.util.Helpers;

public class KickerSubsystem extends ThunderSubsystem {
    private TalonFX m_motor;

    DoubleSupplier optimalSpeedSupplier = () -> 0;

    public KickerSubsystem() {
        TalonFXConfiguration kickerConfig = new TalonFXConfiguration();
        kickerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
        kickerConfig.MotionMagic.MotionMagicAcceleration = Constants.Storage.Kicker.KickerMotionMagic.kAccel;
        kickerConfig.MotionMagic.MotionMagicJerk = Constants.Storage.Kicker.KickerMotionMagic.kJerk;
        kickerConfig.Slot0 = new Slot0Configs()
            .withKP(KickerPID.kP).withKI(KickerPID.kI).withKD(KickerPID.kD)
            .withKS(KickerPID.kS).withKV(KickerPID.kV).withKA(KickerPID.kA);
        kickerConfig.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true)
            );

        if (!Broken.kickerDisabled) {
            m_motor = new TalonFX(Constants.IOMap.Storage.kKickerMotor);
            m_motor.getConfigurator().apply(kickerConfig);
        } else {
            m_motor = null;
        }
    }

    @Override
    public void periodic() {
        if (Broken.kickerDisabled) return;

        SmartDashboard.putNumber("Kicker / Speed RPM", Helpers.RPStoRPM(m_motor.getVelocity().getValueAsDouble()));
        SmartDashboard.putNumber("Kicker / Target RPM", Helpers.RPStoRPM(m_motor.getClosedLoopReference().getValueAsDouble()));
        SmartDashboard.putNumber("Kicker / PID Setpoint", m_motor.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("Kicker / PID Output", m_motor.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putBoolean("Kicker / At Target RPM", atRPM());
        SmartDashboard.putNumber("Kicker / Output %", m_motor.get());

        SmartDashboard.putNumber("SOTM / Kicker RPM", optimalSpeedSupplier.getAsDouble());
    }

    public void setOptimalSpeedGetter(DoubleSupplier supplier) {
        optimalSpeedSupplier = supplier;
    }

    public Command run() {
        if (Broken.kickerDisabled) return CommandBuilder.none(this);
        return this.manual(() -> 1.0);
        // return new CommandBuilder(this)
        //     .onExecute(() -> m_motor.setControl(new VelocityVoltage(Helpers.RPMtoRPS(optimalSpeedSupplier.getAsDouble()))))
        //     .onEnd(m_motor::stopMotor);
    }

    public Command halt() {
        if (Broken.kickerDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onExecute(m_motor::stopMotor);
    }

    @Override
    public Command manual(DoubleSupplier speed) {
        if (Broken.kickerDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(speed.getAsDouble()))
            .onEnd(() -> m_motor.stopMotor());
    }

    public Status status() {
        if (Broken.kickerDisabled) return Status.DISABLED;
        if (!Helpers.onCANChain(m_motor)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_motor)) return Status.ACTIVE;
        return Status.IDLE;
    }

    public boolean atRPM() {
        return Math.abs(Constants.Storage.Kicker.kTargetKickerRPM - Helpers.RPStoRPM(m_motor.getVelocity().getValueAsDouble())) < Constants.Storage.Kicker.kRPMTolerance;
    }
}
