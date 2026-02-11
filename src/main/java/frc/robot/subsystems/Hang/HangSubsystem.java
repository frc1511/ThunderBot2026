package frc.robot.subsystems.Hang;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Constants.HangConstants;
import frc.util.Constants.IOMap;

public class HangSubsystem extends SubsystemBase {
    private SparkMax m_motor;
    private RelativeEncoder m_encoder;
    private SparkClosedLoopController m_pidController;

    private DigitalInput m_lowerLimitSensor;
    private DigitalInput m_upperLimitSensor;
    private boolean m_isZeroed;

    public HangSubsystem() {
        m_motor = new SparkMax(IOMap.Hang.hangMotor, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        motorConfig.closedLoop
            .pid(0.09, 0, 0) // TODO: Move to Constants when we merge back 
            .allowedClosedLoopError(Constants.HangConstants.kSetpointPositionTolerance, ClosedLoopSlot.kSlot0)
            .outputRange(Constants.HangConstants.kMaxPullSpeed, Constants.HangConstants.kMaxDeploySpeed);

        m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getClosedLoopController();
        

        m_lowerLimitSensor = new DigitalInput(IOMap.Hang.lowerLimit);
        m_upperLimitSensor = new DigitalInput(IOMap.Hang.upperLimit);

        m_isZeroed = false;
    }

    public void periodic() {
        SmartDashboard.putBoolean("Hang_atLower", isAtLowerLimit());
        SmartDashboard.putBoolean("Hang_atUpper", isAtUpperLimit());
        SmartDashboard.putBoolean("Hang_isZeroed", isZeroed());
        SmartDashboard.putNumber("Hang_position", m_encoder.getPosition());
        SmartDashboard.putNumber("Hang_out", m_motor.getAppliedOutput());
        SmartDashboard.putNumber("Hang_pidSetpoint", m_pidController.getSetpoint());
        SmartDashboard.putBoolean("Hang_atSetpoint", atSetpoint());
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
                if (isAtLowerLimit()) {
                    m_motor.stopMotor();
                    return;
                }
                m_motor.set(HangConstants.kZeroingSpeed);
            })
            .isFinished(this::isAtLowerLimit)
            .onEnd((boolean interupped) -> {
                m_motor.stopMotor();
                if (!interupped) {
                    m_encoder.setPosition(0);
                    m_isZeroed = true;
                }
            });
    }

    private boolean atSetpoint() {
        return m_pidController.isAtSetpoint() && Math.abs(m_motor.getAppliedOutput()) < Constants.HangConstants.kSetpointMaxVelocity;
    }
    
    private boolean atExtentionLimit() {
        return (atSetpoint() && m_pidController.getSetpoint() == HangConstants.kMaxDeployDistanceRotations) || isAtUpperLimit();
    }

    // TODO: Use PID to automatically extend to the proper height with proper speeds and error correction
    public Command extend() { //hang will die if it goes past the upper limit
        return new CommandBuilder(this)
            .onExecute(() -> {
                if (atExtentionLimit()) {
                    m_motor.stopMotor();
                    return;
                }
                m_pidController.setSetpoint(HangConstants.kMaxDeployDistanceRotations, ControlType.kPosition);
            })
            .isFinished(this::atExtentionLimit)
            .onlyIf(this::isZeroed);
    }

    private boolean atRetractionLimit() {
        return (atSetpoint() && m_pidController.getSetpoint() == HangConstants.kMaxPullDistanceRotations) || isAtLowerLimit();
    }

    // TODO: Use PID to automatically pull to the proper height with proper speeds and error correction
    public Command retract() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_pidController.setSetpoint(HangConstants.kMaxPullDistanceRotations, ControlType.kPosition);
                if (atRetractionLimit()) {
                    m_motor.stopMotor();
                    return;
                }
            })
            .isFinished(this::atRetractionLimit)
            .onlyIf(this::isZeroed);
    }

    public Command halt() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.stopMotor();
            })
            .isFinished(true);
    }

    public Command manual(DoubleSupplier speedSupplier) {
        return new CommandBuilder(this)
            .onExecute(() -> {
                double speed = -speedSupplier.getAsDouble() * .5;
                if ((isAtLowerLimit() && speed < 0) || (isAtUpperLimit() && speed > 0))
                    m_motor.stopMotor();
                else
                    m_motor.set(speed);
            })
            .isFinished(true)
            .repeatedly();
    }
}
