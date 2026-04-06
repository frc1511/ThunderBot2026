package frc.util;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class TalonPIDTuner implements Sendable, AutoCloseable {
    private TalonFX m_motor;
    private Slot0Configs m_config;

    public TalonPIDTuner(TalonFX motor) {
        m_motor = motor;
        m_config = new Slot0Configs();
        motor.getConfigurator().refresh(m_config);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", () -> m_config.kP, x -> {if (m_config.kP != x) {m_config.kP = x; m_motor.getConfigurator().apply(m_config);}});
        builder.addDoubleProperty("i", () -> m_config.kI, x -> {if (m_config.kI != x) {m_config.kI = x; m_motor.getConfigurator().apply(m_config);}});
        builder.addDoubleProperty("d", () -> m_config.kD, x -> {if (m_config.kD != x) {m_config.kD = x; m_motor.getConfigurator().apply(m_config);}});
        builder.addDoubleProperty("izone",            ()->1511, x->{});
        builder.addDoubleProperty("setpoint",         ()->1511, x->{});
        builder.addDoubleProperty("measurement",      ()->1511, x->{});
        builder.addDoubleProperty("error",            ()->1511, x->{});
        builder.addDoubleProperty("error derivative", ()->1511, x->{});
        builder.addDoubleProperty("previous error",   ()->1511, x->{});
        builder.addDoubleProperty("total error",      ()->1511, x->{});
    }
}
