package frc.robot.subsystems.Storage;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.CommandBuilder;
import frc.util.Constants;

public class KickerSubsystem extends SubsystemBase {
    private TalonFX m_motor;
    private AnalogInput m_p;
    public KickerSubsystem() {
        m_motor = new TalonFX(Constants.IOMap.Storage.kKickerMotor);
        m_p = new AnalogInput(0);
    }

    public Command playSoccer() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.set(m_p.getVoltage() / 5);
            })
            .isFinished(() -> false);
    }

    public void periodic() {
        SmartDashboard.putNumber("kickerPercent", m_p.getVoltage() / 5);
    }

    public Command halt() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.stopMotor();
            });
    }

    public Command maunal_kicker(DoubleSupplier speed) {
        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(speed.getAsDouble()));
    }
}
