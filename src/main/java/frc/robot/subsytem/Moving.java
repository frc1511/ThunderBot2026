package frc.robot.subsytem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.CommandBuilder;

public class Moving extends SubsystemBase {
    private TalonFX m_rmotor;
    private TalonFX m_lmotor;
    private AnalogInput m_potentiomiter;

    public Moving() {
        m_rmotor = new TalonFX(1);
        m_lmotor = new TalonFX(0);
        m_potentiomiter = new AnalogInput(0);
    }

    private double getRunSpeed() {
        // return m_potentiomiter.getVoltage() / 5;
        return .50;
    }

    public void periodic() {
        SmartDashboard.putNumber("LE SPEED", getRunSpeed());
    }

    public Command runForward() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_rmotor.set(getRunSpeed());
                m_lmotor.set(-getRunSpeed());
            });
    }

    public Command runBackward() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_rmotor.set(-getRunSpeed());
                m_lmotor.set(getRunSpeed());
            });
    }

    public Command stop() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_rmotor.set(0);
                m_lmotor.set(0);
            });
    }
}