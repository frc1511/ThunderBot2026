package frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.CommandBuilder;
import frc.util.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax m_chompMotor;

    public IntakeSubsystem() {
        m_chompMotor = new SparkMax(Constants.IOMap.Intake.chompMotor, MotorType.kBrushless);
    }

    /**
     * Intake
     */
    public Command eat() {
        return new CommandBuilder(this)
            .onExecute(() -> m_chompMotor.set(Constants.Hunger.Intake.kEatSpeed))
            .isFinished(true)
            .withName(Constants.Hunger.Intake.intakeCommandName);
    }

    /**
     * Stop Intake
     */
    public Command stopEating() {
        return new CommandBuilder(this)
            .onExecute(m_chompMotor::stopMotor)
            .isFinished(true);
    }

    public Command manual_eating(DoubleSupplier speed) {
        return new CommandBuilder(this)
            .onExecute(() -> m_chompMotor.set(speed.getAsDouble()));
    }
}
