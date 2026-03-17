package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Constants.Status;
import frc.util.Thunder.ThunderSubsystem;
import frc.util.Helpers;

public class IntakeSubsystem extends ThunderSubsystem {
    private TalonFX m_motor;

    public IntakeSubsystem() {
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeConfig.Slot0 = new Slot0Configs()
            .withKP(Constants.Hunger.Intake.IntakePID.kP).withKI(Constants.Hunger.Intake.IntakePID.kI).withKD(Constants.Hunger.Intake.IntakePID.kD);
        intakeConfig.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(30))
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true)
            );
        
        if (!Broken.intakeDisabled) {
            m_motor = new TalonFX(Constants.IOMap.Intake.kChompMotor);
            m_motor.getConfigurator().apply(intakeConfig);
        } else {
            m_motor = null;
        }
    }

    @Override
    public void periodic() {
        if (Broken.intakeDisabled) return;

        SmartDashboard.putNumber("intake_rpm", Helpers.RPStoRPM(m_motor.getVelocity().getValueAsDouble()));
        SmartDashboard.putNumber("intake_target_rpm", Helpers.RPStoRPM(m_motor.getClosedLoopReference().getValueAsDouble()));
        SmartDashboard.putNumber("intake_output_V", m_motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("intake_err", Math.abs(Helpers.RPStoRPM(m_motor.getVelocity().getValueAsDouble()) - Constants.Hunger.Intake.kEatRPM));
    }

    /**
     * Intake
     */
    public Command eat() {
        if (Broken.intakeDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> {
                // m_motor.setControl(new VelocityVoltage(Helpers.RPMtoRPS(Constants.Hunger.Intake.kEatRPM)));
                m_motor.set(Constants.Hunger.Intake.kEatPercent);
            })
            .onEnd(m_motor::stopMotor)
            .withName(Constants.Hunger.Intake.intakeCommandName);
    }

    /**
     * Intake
     */
    public Command eatStart() {
        if (Broken.intakeDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> {
                // m_motor.setControl(new VelocityVoltage(Helpers.RPMtoRPS(Constants.Hunger.Intake.kEatRPM)));
                m_motor.set(Constants.Hunger.Intake.kEatPercent);
            })
            .isFinished(true)
            .withName(Constants.Hunger.Intake.intakeCommandName);
    }

    // Outtake
    public Command outtake() {
        if (Broken.intakeDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> {
                // m_motor.setControl(new VelocityVoltage(-Helpers.RPMtoRPS(Constants.Hunger.Intake.kEatRPM)));
                m_motor.set(-Constants.Hunger.Intake.kEatPercent);
            })
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
