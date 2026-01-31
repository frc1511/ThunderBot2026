package frc.robot.subsystems.Storage;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Alert;
import frc.util.CommandBuilder;
import frc.util.Constants;

public class SpindexerSubsystem extends SubsystemBase {
    private TalonFX m_motor;
    private Timer m_timer;

    public SpindexerSubsystem() {
        m_motor = new TalonFX(Constants.IOMap.Storage.kSpindexerMotor);
        m_motor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

        m_timer = new Timer();
        m_timer.stop();
        m_timer.reset();
    }

    public static interface SpinDuration { // Seconds
        double FULL_BAY = 4;
        double PARTIAL_BAY = 2;
        double AUTO_BAY = 1;
        double INTAKE = 10_000_000;
    }
    
    /**
     * PLEASE use {@code SpinDuration} for the duration
    */
    public Command spin(double duration) {
        return new CommandBuilder(this)
            .onInitialize(() -> {
                if (m_timer.isRunning()) Alert.warning("Overwriting spin duration, please wait next time :P");
                m_timer.restart();
            })
            .onExecute(() -> {
                m_motor.set(1);
            })
            .isFinished(() -> 
                m_timer.get() > duration
            );
    }

    public Command manual_run(DoubleSupplier speed) {
        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(speed.getAsDouble()));
    }
}
