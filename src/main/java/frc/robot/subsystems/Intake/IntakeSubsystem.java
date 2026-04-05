package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Constants.Status;
import frc.util.Thunder.ThunderSubsystem;
import frc.util.Helpers;

public class IntakeSubsystem extends ThunderSubsystem {
    private TalonFX m_motorLeft;
    private TalonFX m_motorRight;

    private TalonFX m_primaryMotor;

    public IntakeSubsystem() {
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(30))
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true)
            );

        if (!Broken.intakeFullyDisabled) {
            if (!Broken.intakeRightDisabled) {
                m_motorRight = new TalonFX(Constants.IOMap.Intake.kChompMotorRight);
                m_motorRight.getConfigurator().apply(intakeConfig);
            }

            if (!Broken.intakeLeftDisabled) {
                m_motorLeft = new TalonFX(Constants.IOMap.Intake.kChompMotorLeft);
                m_motorLeft.getConfigurator().apply(intakeConfig);
                if (Broken.intakeRightDisabled) {
                    m_primaryMotor = m_motorLeft;
                } else {
                    m_motorLeft.setControl(new Follower(Constants.IOMap.Intake.kChompMotorRight, MotorAlignmentValue.Opposed));
                    m_primaryMotor = m_motorRight;
                }
            }
        } else {
            Broken.intakeFullyDisabled = true;
            m_primaryMotor = null;
        }
    }

    @Override
    public void periodic() {
        if (Broken.intakeFullyDisabled) return;

        SmartDashboard.putNumber("intake_rpm", Helpers.RPStoRPM(m_primaryMotor.getVelocity().getValueAsDouble()));
        SmartDashboard.putNumber("intake_target_rpm", Helpers.RPStoRPM(m_primaryMotor.getClosedLoopReference().getValueAsDouble()));
        SmartDashboard.putNumber("intake_output_V", m_primaryMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("intake_err", Math.abs(Helpers.RPStoRPM(m_primaryMotor.getVelocity().getValueAsDouble()) - Constants.Hunger.Intake.kEatRPM));
    }

    /**
     * Intake
     */
    public Command eat() {
        if (Broken.intakeFullyDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onExecute(() -> {
                m_primaryMotor.set(Constants.Hunger.Intake.kEatPercent);
            })
            .onEnd(m_primaryMotor::stopMotor)
            .withName(Constants.Hunger.Intake.intakeCommandName);
    }

    /**
     * Intake
     */
    public Command eatStart() {
        if (Broken.intakeFullyDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onExecute(() -> {
                m_primaryMotor.set(Constants.Hunger.Intake.kEatPercent);
            })
            .isFinished(true)
            .withName(Constants.Hunger.Intake.intakeCommandName);
    }

    /** 
     * Outtake
     */
    public Command outtake() {
        if (Broken.intakeFullyDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onExecute(() -> {
                m_primaryMotor.set(-Constants.Hunger.Intake.kEatPercent);
            })
            .onEnd(m_primaryMotor::stopMotor)
            .withName(Constants.Hunger.Intake.intakeCommandName);
    }

    /**
     * Stop Intake
     */
    public Command stopEating() {
        if (Broken.intakeFullyDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onExecute(m_primaryMotor::stopMotor)
            .isFinished(true);
    }

    public Command manual_intakeLeft(DoubleSupplier speed) {
        if (Broken.intakeFullyDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onExecute(() -> m_motorLeft.set(speed.getAsDouble()))
            .onEnd(() -> m_motorLeft.stopMotor());
    }

    public Command manual_intakeRight(DoubleSupplier speed) {
        if (Broken.intakeFullyDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onExecute(() -> m_motorRight.set(speed.getAsDouble()))
            .onEnd(() -> m_motorRight.stopMotor());
    }

    public Status status() {
        if (Broken.intakeFullyDisabled) return Status.DISABLED;
        if (!Helpers.onCANChain(m_primaryMotor)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_primaryMotor)) return Status.ACTIVE;
        return Status.IDLE;
    }
}
