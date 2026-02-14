package frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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
    private TalonFX m_motor;

    public PivotSubsystem() {
        Slot0Configs pivotConfig = new Slot0Configs()
            .withKP(Constants.Hunger.Pivot.PivotPID.kP).withKI(Constants.Hunger.Pivot.PivotPID.kI).withKD(Constants.Hunger.Pivot.PivotPID.kD);

        if (!Broken.pivotDisabled) {
            m_motor = new TalonFX(Constants.IOMap.Intake.pivotMotor);
            m_motor.getConfigurator().apply(pivotConfig);
        } else {
            m_motor = null;
        }
    }

    public Command pivotDown() {
        if (Broken.pivotDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_motor.setControl(new PositionVoltage(Constants.Hunger.Pivot.Position.BOTTOM.get())))
            .isFinished(() -> m_motor.getClosedLoopError().getValueAsDouble() < Constants.Hunger.Pivot.kTolerance);
    }

    public Command pivotUp() {
        if (Broken.pivotDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_motor.setControl(new PositionVoltage(Constants.Hunger.Pivot.Position.TOP.get())))
            .isFinished(() -> m_motor.getClosedLoopError().getValueAsDouble() < Constants.Hunger.Pivot.kTolerance);
    }

    public Command manual_pivot(DoubleSupplier speed) {
        if (Broken.pivotDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(speed.getAsDouble()));
    }

    public Status status() {
        if (Broken.pivotDisabled) return Status.DISABLED;
        if (!Helpers.onCANChain(m_motor)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_motor)) return Status.ACTIVE;
        return Status.IDLE;
    }
}
