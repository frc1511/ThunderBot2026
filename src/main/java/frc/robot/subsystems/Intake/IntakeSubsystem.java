package frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax m_motor;

    public IntakeSubsystem() {
        if (!Broken.intakeDisabled) {
            m_motor = new SparkMax(Constants.IOMap.Intake.chompMotor, MotorType.kBrushless);
        } else {
            m_motor = null;
        }
    }

    /**
     * Intake
     */
    public Command eat() {
        if (!Broken.intakeDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(Constants.Hunger.Intake.kEatSpeed))
            .isFinished(true)
            .withName(Constants.Hunger.Intake.intakeCommandName);
    }

    /**
     * Stop Intake
     */
    public Command stopEating() {
        if (!Broken.intakeDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(m_motor::stopMotor)
            .isFinished(true);
    }

    public Command manual_eating(DoubleSupplier speed) {
        if (!Broken.intakeDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_motor.set(speed.getAsDouble()));
    }
}
