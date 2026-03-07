package frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Constants.Status;
import frc.util.Thunder.ThunderSubsystem;
import frc.util.Helpers;

public class PivotSubsystem extends ThunderSubsystem {
    private SparkMax m_motor;
    private CANcoder m_CANcoder;
    private SparkMaxConfig pivotConfig;
    private RelativeEncoder m_builtinEncoder;
    private SparkClosedLoopController m_pidController;

    public PivotSubsystem() {
        pivotConfig = new SparkMaxConfig(); 
        pivotConfig.closedLoop
            .pid(Constants.Hunger.Pivot.PivotPID.kP, Constants.Hunger.Pivot.PivotPID.kI, Constants.Hunger.Pivot.PivotPID.kD)
            .allowedClosedLoopError(Constants.Hunger.Pivot.kTolerance, ClosedLoopSlot.kSlot0);
        // .feedForward
        //     .kCosRatio(Constants.Hunger.Pivot.kCosRatio)
        //     .kCos(Constants.Hunger.Pivot.PivotPID.kCos);

        pivotConfig.encoder
            .positionConversionFactor(1d/96d);
        
        if (!Broken.pivotDisabled) {
            m_CANcoder = new CANcoder(Constants.IOMap.Intake.kCANcoder);
            m_CANcoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(Constants.Hunger.Pivot.kCANcoderOffset));

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
        SmartDashboard.putNumber("Pivot_pidError", m_pidController.getSetpoint() - m_CANcoder.getPosition().getValueAsDouble());
    }

    public void setMotorMode(IdleMode mode) {
        if (Broken.pivotDisabled) return; 

        pivotConfig.idleMode(mode);
        m_motor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command halt() {
        if (Broken.pivotDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(m_motor::stopMotor);
    }

    public Command down() {
        if (Broken.pivotDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_pidController.setSetpoint(Constants.Hunger.Pivot.Position.BOTTOM.get(), ControlType.kPosition))
            .isFinished(() -> m_pidController.isAtSetpoint() && Helpers.ensureTarget(Constants.Hunger.Pivot.Position.BOTTOM.get(), m_pidController.getSetpoint(), Constants.Hunger.Pivot.kTolerance))
            .onEnd(() -> m_motor.stopMotor());
    }

    public Command up() {
        if (Broken.pivotDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_pidController.setSetpoint(Constants.Hunger.Pivot.Position.TOP.get(), ControlType.kPosition))
            .isFinished(() -> m_pidController.isAtSetpoint() && Helpers.ensureTarget(Constants.Hunger.Pivot.Position.TOP.get(), m_pidController.getSetpoint(), Constants.Hunger.Pivot.kTolerance));
    }

    public Command upSoft() {
        if (Broken.pivotDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_pidController.setSetpoint(Constants.Hunger.Pivot.Position.TOP.get(), ControlType.kPosition))
            .isFinished(true);
    }

    public Command downSoft() {
        if (Broken.pivotDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_pidController.setSetpoint(Constants.Hunger.Pivot.Position.BOTTOM.get(), ControlType.kPosition))
            .isFinished(true);
    }

    public Command middle() {
        if (Broken.pivotDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_pidController.setSetpoint(Constants.Hunger.Pivot.Position.MIDDLE.get(), ControlType.kPosition))
            .isFinished(() -> m_pidController.isAtSetpoint() && Helpers.ensureTarget(Constants.Hunger.Pivot.Position.MIDDLE.get(), m_pidController.getSetpoint(), Constants.Hunger.Pivot.kTolerance)); // This extra NONSENSE is because the motor controller is slow or something and, uh, doesn't actually set the setpoint for a bit.
    }

    public Command jostle() {
        if (Broken.pivotDisabled) return Commands.none();

        return middle()
            .andThen(down());
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

    public boolean isIn() {
        if (Broken.pivotDisabled) return true;

        return Helpers.ensureTarget(Constants.Hunger.Pivot.Position.TOP.get(), m_builtinEncoder.getPosition(), 0.1);
    }

    public Status status() {
        if (Broken.pivotDisabled) return Status.DISABLED;
        if (!Helpers.onCANChain(m_motor)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_motor)) return Status.ACTIVE;
        return Status.IDLE;
    }
}
