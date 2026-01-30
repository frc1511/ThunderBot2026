package frc.robot.subsytem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.CommandBuilder;

public class Moving extends SubsystemBase {
    private TalonFX m_motor;
    private AnalogInput m_potentiomiter;

    public Moving() {
        m_motor = new TalonFX(41);
        m_potentiomiter = new AnalogInput(0);
    }

    private double getRunSpeed() {
        return m_potentiomiter.getVoltage() / 5;
    }

    public void periodic() {
        SmartDashboard.putNumber("LE SPEED", getRunSpeed());
    }

    public Command runForward() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.set(getRunSpeed());
            });
    }

    public Command runBackward() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.set(-getRunSpeed());
            });
    }

    public Command stop() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.set(0);
            });
    }
}