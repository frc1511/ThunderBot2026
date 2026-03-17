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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Constants.Status;
import frc.util.Constants.Storage.Kicker.KickerPID;
import frc.util.Thunder.Modifiable;
import frc.util.Thunder.ThunderSubsystem;
import frc.util.Helpers;

public class KickerSubsystem extends ThunderSubsystem {
    private TalonFX m_motor;

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

        new Modifiable("optimalRPM", this, () -> Constants.Storage.Kicker.kTargetKickerRPM);
    }

    @Override
    public void periodic() {
        if (Broken.kickerDisabled) return;

        SmartDashboard.putNumber("kicker_vel_rpm", Helpers.RPStoRPM(m_motor.getVelocity().getValueAsDouble()));
        SmartDashboard.putNumber("kicker_target_rpm", Helpers.RPStoRPM(m_motor.getClosedLoopReference().getValueAsDouble()));
        SmartDashboard.putNumber("kicker_pid_setpoint", m_motor.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("kicker_pid_out", m_motor.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putBoolean("kicker_at_target_rpm", atRPM());
        SmartDashboard.putNumber("kicker_%_out", m_motor.get());
    }

    public Command run() {
        if (Broken.kickerDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_motor.setControl(new VelocityVoltage(Helpers.RPMtoRPS((Double) getField("optimalRPM").getValue()))))
            .onEnd(m_motor::stopMotor);
    }

    public Command halt() {
        if (Broken.kickerDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(m_motor::stopMotor);
    }

    public Command manual_kicker(DoubleSupplier speed) {
        if (Broken.kickerDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(speed.getAsDouble()));
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
