package frc.robot.subsystems.Storage;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Constants.Status;
import frc.util.Helpers;

public class SpindexerSubsystem extends SubsystemBase {
    private TalonFX m_motor;

    public SpindexerSubsystem() {
        if (!Broken.spindexerDisabled) {
            m_motor = new TalonFX(Constants.IOMap.Storage.kSpindexerMotor);
            m_motor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        } else {
            m_motor = null;
        }
    }

    /**
     * PLEASE use {@code SpinDuration} for the duration
    */
    public Command spin(double duration) {
        if (Broken.spindexerDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.set(Constants.Storage.Spindexer.kSpeed);
            })
            .onEnd(m_motor::stopMotor)
            .withTimeout(duration);
    }

    public Command manual_spindexer(DoubleSupplier speed) {
        if (Broken.spindexerDisabled) return Commands.none();
        
        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(speed.getAsDouble()));
    }
    
    public Status status() {
        if (Broken.spindexerDisabled) return Status.DISABLED;
        if (!Helpers.onCANChain(m_motor)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_motor)) return Status.ACTIVE;
        return Status.ACTIVE;
    }
}
