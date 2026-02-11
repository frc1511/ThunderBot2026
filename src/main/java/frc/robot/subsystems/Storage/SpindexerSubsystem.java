package frc.robot.subsystems.Storage;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.util.CommandBuilder;
import frc.util.Constants;

public class SpindexerSubsystem extends SubsystemBase {
    private TalonFX m_motor;

    public SpindexerSubsystem() {
        m_motor = new TalonFX(Constants.IOMap.Storage.kSpindexerMotor);
        m_motor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
    }

    /**
     * PLEASE use {@code SpinDuration} for the duration
    */
    public Command spin(double duration) {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.set(Constants.Storage.Spindexer.kSpeed);
            })
            .onEnd(m_motor::stopMotor)
            .withTimeout(duration);
    }

    public Command manual_spindexer(DoubleSupplier speed) {
        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(speed.getAsDouble()));
    }
}
