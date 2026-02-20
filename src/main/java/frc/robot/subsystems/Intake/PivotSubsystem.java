package frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Constants.Status;
import frc.util.Helpers;
import frc.util.ThunderSubsystem;

public class PivotSubsystem extends SubsystemBase implements ThunderSubsystem {
    private SparkMax m_motor;
    private CANcoder m_CANcoder;
    private RelativeEncoder m_builtinEncoder;
    private SparkClosedLoopController m_pidController;

    public PivotSubsystem() {
        SparkMaxConfig pivotConfig = new SparkMaxConfig(); 
        pivotConfig.closedLoop
            .pid(Constants.Hunger.Pivot.PivotPID.kP, Constants.Hunger.Pivot.PivotPID.kI, Constants.Hunger.Pivot.PivotPID.kD)
            .allowedClosedLoopError(Constants.Hunger.Pivot.kTolerance, ClosedLoopSlot.kSlot0)
            .feedForward
                .kCosRatio(Constants.Hunger.Pivot.kCosRatio)
                .kCos(Constants.Hunger.Pivot.PivotPID.kCos);

        pivotConfig.encoder
            .positionConversionFactor(1/96);

        if (!Broken.pivotDisabled) {
            m_CANcoder = new CANcoder(Constants.IOMap.Intake.kCANcoder);

            m_motor = new SparkMax(Constants.IOMap.Intake.kPivotMotor, MotorType.kBrushless);
            m_motor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            m_builtinEncoder = m_motor.getEncoder();
            m_builtinEncoder.setPosition(m_CANcoder.getPosition().getValueAsDouble());

            m_pidController = m_motor.getClosedLoopController();
        } else {
            m_motor = null;
            m_CANcoder = null;
            m_builtinEncoder = null;
            m_pidController = null;
        }
    }

    @Override
    public void periodic() {
        if (Broken.pivotDisabled) return;
        
        m_builtinEncoder.setPosition(m_CANcoder.getPosition().getValueAsDouble());
        
        SmartDashboard.putNumber("Pivot_builtin_position", m_builtinEncoder.getPosition());
        SmartDashboard.putNumber("Pivot_CANcoder_position", m_CANcoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot_pidSetpoint", m_pidController.getSetpoint());
        SmartDashboard.putBoolean("Pivot_atSetpoint", m_pidController.isAtSetpoint());
        SmartDashboard.putNumber("Pivot_output_%", m_motor.getAppliedOutput());
        SmartDashboard.putNumber("Pivot_output_A", m_motor.getOutputCurrent());
    }

    public Command halt() {
        if (Broken.pivotDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(m_motor::stopMotor);
    }

    public Command pivotDown() {
        if (Broken.pivotDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_pidController.setSetpoint(Constants.Hunger.Pivot.Position.BOTTOM.get(), ControlType.kPosition))
            .isFinished(m_pidController::isAtSetpoint);
    }

    public Command pivotUp() {
        if (Broken.pivotDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_pidController.setSetpoint(Constants.Hunger.Pivot.Position.TOP.get(), ControlType.kPosition))
            .isFinished(m_pidController::isAtSetpoint);
    }

    public Command manual_pivot(DoubleSupplier speed) {
        if (Broken.pivotDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(speed.getAsDouble()));
    }

    public Command manual_voltage(DoubleSupplier voltage) {
        if (Broken.pivotDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_motor.setVoltage(voltage.getAsDouble()));
    }

    public Status status() {
        if (Broken.pivotDisabled) return Status.DISABLED;
        if (!Helpers.onCANChain(m_motor)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_motor)) return Status.ACTIVE;
        return Status.IDLE;
    }
}
