package frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

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

public class IntakeSubsystem extends SubsystemBase implements ThunderSubsystem {
    private TalonFX m_motor;

    public IntakeSubsystem() {
        if (!Broken.intakeDisabled) {
            m_motor = new TalonFX(Constants.IOMap.Intake.kChompMotor);
        } else {
            m_motor = null;
        }
    }

    @Override
    public void periodic() {
        if (Broken.intakeDisabled) return;

        // SmartDashboard.putNumber("Intake_output_%", m_motor.getAppliedOutput());
        // SmartDashboard.putNumber("Intake_output_A", m_motor.getOutputCurrent());
    }

    /**
     * Intake
     */
    public Command eat() {
        if (Broken.intakeDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(Constants.Hunger.Intake.kEatSpeed))
            .onEnd(m_motor::stopMotor)
            .withName(Constants.Hunger.Intake.intakeCommandName);
    }

    /**
     * Stop Intake
     */
    public Command stopEating() {
        if (Broken.intakeDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(m_motor::stopMotor)
            .isFinished(true);
    }

    public Command manual_intake(DoubleSupplier speed) {
        if (Broken.intakeDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(speed.getAsDouble()));
    }

    
    public Status status() {
        if (Broken.intakeDisabled) return Status.DISABLED;
        if (!Helpers.onCANChain(m_motor)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_motor)) return Status.ACTIVE;
        return Status.IDLE;
    }
}
