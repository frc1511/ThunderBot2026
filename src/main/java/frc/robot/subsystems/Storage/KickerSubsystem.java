package frc.robot.subsystems.Storage;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Constants.Status;
import frc.util.Helpers;
import frc.util.ThunderSubsystem;

public class KickerSubsystem extends SubsystemBase implements ThunderSubsystem {
    private TalonFX m_motor;

    public KickerSubsystem() {
        if (!Broken.kickerDisabled) {
            m_motor = new TalonFX(Constants.IOMap.Storage.kKickerMotor);
        } else {
            m_motor = null;
        }
    }

    public Command playSoccer() {
        if (Broken.kickerDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(Constants.Storage.Kicker.kSpeed))
            .onEnd(m_motor::stopMotor);
    }

    public Command halt() {
        if (Broken.kickerDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(m_motor::stopMotor);
    }

    public Command manual(DoubleSupplier speed) {
        if (Broken.kickerDisabled) return Commands.none();
        
        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(speed.getAsDouble()));
    }
    
    public Status status() {
        if (Broken.kickerDisabled) return Status.DISABLED;
        if (!Helpers.onCANChain(m_motor)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_motor)) return Status.ACTIVE;
        return Status.ACTIVE;
    }
}
