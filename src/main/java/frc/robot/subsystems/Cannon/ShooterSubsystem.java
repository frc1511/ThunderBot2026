package frc.robot.subsystems.Cannon;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Helpers;
import frc.util.ThunderSubsystem;
import frc.util.Constants.Status;

public class ShooterSubsystem extends SubsystemBase implements ThunderSubsystem {
    private TalonFX m_shooterMotorA;
    private TalonFX m_shooterMotorB;

    private TalonFX m_primaryMotor;

    public ShooterSubsystem() {
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.Slot0 = new Slot0Configs()
            .withKP(Constants.Shooter.ShooterPID.kP).withKI(Constants.Shooter.ShooterPID.kI).withKD(Constants.Shooter.ShooterPID.kD);
        
        if (!Broken.shooterFullyDisabled) {
            if (!Broken.shooterADisabled) {
                m_shooterMotorA = new TalonFX(Constants.IOMap.Shooter.kShooterMotorA);
                m_shooterMotorA.getConfigurator().apply(shooterConfig);
            }
    
            if (!Broken.shooterBDisabled) {
                m_shooterMotorB = new TalonFX(Constants.IOMap.Shooter.kShooterMotorB);
                m_shooterMotorB.getConfigurator().apply(shooterConfig);
                if (Broken.shooterADisabled) {
                    // Where A is broken, B needs to be the primary motor
                    m_primaryMotor = m_shooterMotorB;
                } else {
                    m_shooterMotorB.setControl(new Follower(Constants.IOMap.Shooter.kShooterMotorA, MotorAlignmentValue.Opposed));
                    m_primaryMotor = m_shooterMotorA;
                }
            }
        } else {
            Broken.shooterFullyDisabled = true;
            m_primaryMotor = null;
        }
    }

    public boolean shooterAtSpeed() {
        if (Broken.shooterFullyDisabled) return true;

        return m_primaryMotor.getClosedLoopError().getValueAsDouble() < Constants.Shooter.kShooterAtSpeedTolerance;
        // return Math.abs(Constants.Shooter.kMaxShooterSpeed - m_primaryMotor.getVelocity().getValueAsDouble()) < Constants.Shooter.kShooterAtSpeedTolerance;
    }

    public void periodic() {
        if (Broken.shooterFullyDisabled) return;
        SmartDashboard.putNumber("Actual Output", m_primaryMotor.getVelocity().getValueAsDouble() * 60);
    }

    public Command stopShooter() {
        if (Broken.shooterFullyDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(m_primaryMotor::stopMotor)
            .isFinished(true);
    }

    public Command preheat() {
        if (Broken.shooterFullyDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> m_primaryMotor.setControl(new VelocityVoltage(Constants.Shooter.kTargetShooterRPM / 60)))
            .isFinished(this::shooterAtSpeed);
    }

    public Command manual_shooter(DoubleSupplier speed) {
        if (Broken.shooterFullyDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> {
                m_primaryMotor.setControl(new VelocityVoltage(speed.getAsDouble() * Constants.Shooter.kTargetShooterRPM / 60));
            })
            .isFinished(() -> false);
    }

    public Command shoot() {
        if (Broken.shooterFullyDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> {
                if (Broken.shooterFullyDisabled) return;
                m_primaryMotor.set(Constants.Shooter.kMaxShooterSpeed);
            })
            .isFinished(true);
    }

    public Status status() {
        if (Broken.shooterFullyDisabled) return Status.DISABLED;
        if (!Helpers.onCANChain(m_primaryMotor)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_primaryMotor)) return Status.ACTIVE;
        return Status.IDLE;
    }
}