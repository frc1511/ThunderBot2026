package frc.robot.subsytems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.CommandBuilder;
import frc.util.Constants.HangConstants;
import frc.util.Constants.IOMap;

public class HangSubsystem extends SubsystemBase {
    private TalonFX m_hangMotor;
    private TalonFXConfiguration m_hangMotorConfiguration;
    private DigitalInput m_lowerLimitSensor;
    private boolean m_isZeroed;

    public HangSubsystem() {
        m_hangMotor = new TalonFX(IOMap.Hang.hangMotor);

        m_hangMotorConfiguration = new TalonFXConfiguration();
        m_hangMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_hangMotor.getConfigurator().apply(m_hangMotorConfiguration);

        m_lowerLimitSensor = new DigitalInput(1);

        m_isZeroed = false;
    }

    private boolean isAtLowerLimit() {
        return m_lowerLimitSensor.get();
    }

    public boolean isZeroed() {
        return m_isZeroed;
    }

    public Command zeroHang() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_hangMotor.set(HangConstants.kZeroingSpeed);
            })
            .isFinished(this::isAtLowerLimit)
            .onEnd((boolean interupped) -> {
                m_hangMotor.set(0);
                if (!interupped) {
                    m_hangMotor.setPosition(0);
                    m_isZeroed = true;
                }
            });
    }

    // TODO: Use PID to automatically extend to the proper height with proper speeds and error correction
    public Command extend() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_hangMotor.set(HangConstants.kMaxDelpoySpeed);
            })
            .isFinished(() -> m_hangMotor.getPosition().getValueAsDouble() > HangConstants.kMaxDeployDistanceRotations)
            .onlyIf(this::isZeroed);
    }

    // TODO: Use PID to automatically pull to the proper height with proper speeds and error correction
    public Command retract() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_hangMotor.set(HangConstants.kMaxPullSpeed);
            })
            .isFinished(() -> m_hangMotor.getPosition().getValueAsDouble() < HangConstants.kMaxPullDistanceRotations)
            .onlyIf(this::isZeroed);
    }
}
