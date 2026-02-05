package frc.robot.subsytems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.util.CommandBuilder;
import frc.util.Constants.HangConstants;
import frc.util.Constants.IOMap;

public class HangSubsystem extends SubsystemBase {
    private SparkMax m_motor;
    private RelativeEncoder m_encoder;
    private DigitalInput m_lowerLimitSensor;
    private DigitalInput m_upperLimitSensor;
    private boolean m_isZeroed;

    public HangSubsystem() {
        m_motor = new SparkMax(IOMap.Hang.hangMotor, MotorType.kBrushless);
        m_motor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_encoder = m_motor.getEncoder();

        m_lowerLimitSensor = new DigitalInput(IOMap.Hang.lowerLimit);
        m_upperLimitSensor = new DigitalInput(IOMap.Hang.upperLimit);

        m_isZeroed = false;
    }

    public void periodic() {
        SmartDashboard.putBoolean("Hang_atLower", isAtLowerLimit());
        SmartDashboard.putBoolean("Hang_atUpper", isAtUpperLimit());
        SmartDashboard.putBoolean("Hang_isZeroed", isZeroed());
        SmartDashboard.putNumber("Hang_position", m_encoder.getPosition());
    }

    private boolean isAtLowerLimit() {
        return !m_lowerLimitSensor.get();
    }

    private boolean isAtUpperLimit() {
        return !m_upperLimitSensor.get();
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
    public Command extend() { //hang will die if it goes past the upper limit
        return new CommandBuilder(this)
            .onExecute(() -> {
                if (isAtUpperLimit()) {m_motor.set(0);return;}
                m_motor.set(HangConstants.kMaxDeploySpeed);
            })
            .isFinished(() -> isAtUpperLimit() || m_encoder.getPosition() > HangConstants.kMaxDeployDistanceRotations)
            .onlyIf(this::isZeroed);
    }

    // TODO: Use PID to automatically pull to the proper height with proper speeds and error correction
    public Command retract() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                if (isAtLowerLimit()) {m_motor.set(0);return;};
                m_motor.set(HangConstants.kMaxPullSpeed);
            })
            .isFinished(() -> m_encoder.getPosition() < HangConstants.kMaxPullDistanceRotations || isAtLowerLimit())
            .onlyIf(this::isZeroed);
    }

    public Command halt() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.set(0);
            })
            .isFinished(true);
    }

    public Command manual(DoubleSupplier speedSupplier) {
        return new CommandBuilder(this)
            .onExecute(() -> {
                double speed = speedSupplier.getAsDouble() * .5;
                if ((isAtLowerLimit() && speed > 0) || (isAtUpperLimit() && speed < 0))
                    m_motor.set(0);
                else
                    m_motor.set(speed);
            })
            .isFinished(true)
            .repeatedly();
    }
}
