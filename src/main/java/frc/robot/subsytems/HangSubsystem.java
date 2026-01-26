package frc.robot.subsytems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.CommandBuilder;
import frc.util.Constants.HangConstants;
import frc.util.Constants.IOMap;

public class HangSubsystem extends SubsystemBase {
    private SparkMax m_motor;
    private RelativeEncoder m_encoder;
    private DigitalInput m_lowerLimitSensor;
    private boolean m_isZeroed;

    public HangSubsystem() {
        m_motor = new SparkMax(IOMap.Hang.hangMotor, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();

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
                m_motor.set(HangConstants.kZeroingSpeed);
            })
            .isFinished(this::isAtLowerLimit)
            .onEnd((boolean interupped) -> {
                m_motor.set(0);
                if (!interupped) {
                    m_encoder.setPosition(0);
                    m_isZeroed = true;
                }
            });
    }

    // TODO: Use PID to automatically extend to the proper height with proper speeds and error correction
    public Command extend() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.set(HangConstants.kMaxDelpoySpeed);
            })
            .isFinished(() -> m_encoder.getPosition() > HangConstants.kMaxDeployDistanceRotations)
            .onlyIf(this::isZeroed);
    }

    // TODO: Use PID to automatically pull to the proper height with proper speeds and error correction
    public Command retract() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.set(HangConstants.kMaxPullSpeed);
            })
            .isFinished(() -> m_encoder.getPosition() < HangConstants.kMaxPullDistanceRotations)
            .onlyIf(this::isZeroed);
    }
}
