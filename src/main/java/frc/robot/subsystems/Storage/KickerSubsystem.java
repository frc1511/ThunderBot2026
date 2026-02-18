package frc.robot.subsystems.Storage;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Constants.Status;
import frc.util.Constants.Storage.Kicker.KickerPID;
import frc.util.Helpers;
import frc.util.ThunderSubsystem;

public class KickerSubsystem extends SubsystemBase implements ThunderSubsystem {
    private TalonFX m_motor;

    public KickerSubsystem() {
        Slot0Configs pidConfig = new Slot0Configs().withKP(KickerPID.kP).withKI(KickerPID.kI).withKD(KickerPID.kD);
        if (!Broken.kickerDisabled) {
            m_motor = new TalonFX(Constants.IOMap.Storage.kKickerMotor);
            m_motor.getConfigurator().apply(pidConfig);
        } else {
            m_motor = null;
        }
    }

    @Override
    public void periodic() {
        if (Broken.kickerDisabled) return;
        
        SmartDashboard.putNumber("kicker_vel_rpm", Helpers.RPStoRPM(m_motor.getVelocity().getValueAsDouble()));
        SmartDashboard.putNumber("kicker_target_rpm", Helpers.RPStoRPM(m_motor.getClosedLoopReference().getValueAsDouble()));
        SmartDashboard.putNumber("kicker_pid_setpoint", m_motor.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("kicker_pid_out", m_motor.getClosedLoopOutput().getValueAsDouble());
    }

    public Command run() {
        if (Broken.kickerDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_motor.setControl(new VelocityVoltage(Helpers.RPMtoRPS(Constants.Storage.Kicker.kTargetKickerRPM))))
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
}
