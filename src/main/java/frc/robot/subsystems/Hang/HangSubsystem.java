package frc.robot.subsystems.Hang;

import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Helpers;
import frc.util.ThunderSubsystem;
import frc.util.Constants.HangConstants;
import frc.util.Constants.IOMap;
import frc.util.Constants.Status;
import frc.util.Constants.HangConstants.HangPID;

public class HangSubsystem extends SubsystemBase implements ThunderSubsystem {
    private SparkMax m_motor;
    private RelativeEncoder m_encoder;
    private SparkClosedLoopController m_pidController;

    private DigitalInput m_lowerLimitSensor;
    private DigitalInput m_upperLimitSensor;
    private boolean m_isZeroed;

    public HangSubsystem() {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        motorConfig.closedLoop
            .pid(HangPID.kP, HangPID.kI, HangPID.kD)
            .allowedClosedLoopError(Constants.HangConstants.kSetpointPositionTolerance, ClosedLoopSlot.kSlot0)
            .outputRange(Constants.HangConstants.kMaxPullSpeed, Constants.HangConstants.kMaxDeploySpeed);
        
        m_isZeroed = false;

        if (!Broken.hangFullyDisabled) {
            m_motor = new SparkMax(IOMap.Hang.kHangMotor, MotorType.kBrushless);
            m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_encoder = m_motor.getEncoder();
            m_pidController = m_motor.getClosedLoopController();
            
            // For broken functionality, these still "work" when the sensors are disconnected so we don't have to null them to avoid errors
            m_lowerLimitSensor = new DigitalInput(IOMap.Hang.kDIOlowerLimit);
            m_upperLimitSensor = new DigitalInput(IOMap.Hang.kDIOupperLimit);
        }

        if (Broken.hangLowerLimitDisabled) {
            m_isZeroed = true;
        }
    }

    @Override
    public void periodic() {
        if (Broken.hangFullyDisabled) return;
        
        SmartDashboard.putBoolean("Hang_atLower", isAtLowerLimit());
        SmartDashboard.putBoolean("Hang_atUpper", isAtUpperLimit());
        SmartDashboard.putBoolean("Hang_isZeroed", isZeroed());
        SmartDashboard.putNumber("Hang_position", m_encoder.getPosition());
        SmartDashboard.putNumber("Hang_output_%", m_motor.getAppliedOutput());
        SmartDashboard.putNumber("Hang_pidSetpoint", m_pidController.getSetpoint());
        SmartDashboard.putBoolean("Hang_atSetpoint", atSetpoint());
        SmartDashboard.putNumber("Hang_output_A", m_motor.getOutputCurrent());
    }

    private boolean isAtLowerLimit() {
        if (Broken.hangLowerLimitDisabled) return false;
        if (Broken.hangFullyDisabled) return true;

        return !m_lowerLimitSensor.get();
    }

    private boolean isAtUpperLimit() {
        if (Broken.hangUpperLimitDisabled) return false;
        if (Broken.hangFullyDisabled) return true;
        
        return !m_upperLimitSensor.get();
    }

    public boolean isZeroed() {
        return m_isZeroed;
    }

    public Command zeroHang() {
        if (Broken.hangFullyDisabled || Broken.hangLowerLimitDisabled) {
            m_isZeroed = true;
            return Commands.none();
        }

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
        if (Broken.hangFullyDisabled) return true;

        return m_pidController.isAtSetpoint() && Math.abs(m_motor.getAppliedOutput()) < Constants.HangConstants.kSetpointMaxVelocity;
    }
    
    private boolean atExtentionLimit() {
        if (Broken.hangFullyDisabled) return true;
        
        return (atSetpoint() && m_pidController.getSetpoint() == HangConstants.kMaxDeployDistanceRotations) || isAtUpperLimit();
    }

    public Command extend() { //hang will die if it goes past the upper limit
        if (Broken.hangFullyDisabled) return Commands.none();
        
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
        if (Broken.hangFullyDisabled) return true;

        return (atSetpoint() && m_pidController.getSetpoint() == HangConstants.kMaxPullDistanceRotations) || isAtLowerLimit();
    }

    public Command retract() {
        if (Broken.hangFullyDisabled) return Commands.none();

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
        if (Broken.hangFullyDisabled) return new InstantCommand(()->{}, this);

        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.stopMotor();
            })
            .isFinished(true);
    }

    public Command manual(DoubleSupplier speedSupplier) {
        if (Broken.hangFullyDisabled) return Commands.none();

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
    
    public Status status() {
        if (Broken.hangFullyDisabled) return Status.DISABLED;
        if (!Helpers.onCANChain(m_motor)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_motor)) return Status.ACTIVE;
        return Status.IDLE;
    }
}
