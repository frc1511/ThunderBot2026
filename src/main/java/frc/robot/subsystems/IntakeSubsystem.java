package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.CommandBuilder;
import frc.util.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX m_pivotMotor;
    private SparkMax m_chompMotor;

    public IntakeSubsystem() {
        m_pivotMotor = new TalonFX(Constants.IOMap.Intake.pivotMotor);
        m_chompMotor = new SparkMax(Constants.IOMap.Intake.chompMotor, MotorType.kBrushless);

        Slot0Configs pivotConfig = new Slot0Configs()
            .withKP(0).withKI(0).withKD(0);
        m_pivotMotor.getConfigurator().apply(pivotConfig);
    }

    public Command eat() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_chompMotor.set(0.3);
            })
            .isFinished(true);
    }

    public Command stopEating() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_chompMotor.stopMotor();
            })
            .isFinished(true);
    }

    public Command pivotDown() {
        return new CommandBuilder(this)
        .onExecute(() -> {
            m_pivotMotor.setControl(new PositionVoltage(40));
        })
        .isFinished(() -> {
            return m_pivotMotor.getClosedLoopError().getValueAsDouble() < 0.5d;
        });
    }

    public Command pivotUp() {
        return new CommandBuilder(this)
        .onExecute(() -> {
            m_pivotMotor.setControl(new PositionVoltage(31));
        })
        .isFinished(() -> {
            return m_pivotMotor.getClosedLoopError().getValueAsDouble() < 0.5d;
        });
    }

    public Command consume() {
        return pivotDown().alongWith(eat());
    }

    public Command excuseYourself() {
        return pivotUp().alongWith(stopEating());
    }
}
