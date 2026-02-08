package frc.robot.subsystems.Storage;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.CommandBuilder;
import frc.util.Constants;

public class KickerSubsystem extends SubsystemBase {
    private TalonFX m_motor;

    public KickerSubsystem() {
        m_motor = new TalonFX(Constants.IOMap.Storage.kKickerMotor);
    }

    public Command playSoccer() {
        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(Constants.Storage.Kicker.kSpeed))
            .onEnd(m_motor::stopMotor);
    }

    public Command halt() {
        return new CommandBuilder(this)
            .onExecute(m_motor::stopMotor);
    }

    public Command manual(DoubleSupplier speed) {
        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(speed.getAsDouble()));
    }
}
