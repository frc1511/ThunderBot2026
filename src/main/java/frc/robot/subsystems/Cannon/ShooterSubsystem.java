package frc.robot.subsystems.Cannon;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.CommandBuilder;
import frc.util.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX m_shooterMotorA;
    private TalonFX m_shooterMotorB;

    public ShooterSubsystem() {
        m_shooterMotorA = new TalonFX(Constants.IOMap.Shooter.kShooterMotorA);
        m_shooterMotorB = new TalonFX(Constants.IOMap.Shooter.kShooterMotorB);
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_shooterMotorA.getConfigurator().apply(shooterConfig);
        m_shooterMotorB.getConfigurator().apply(shooterConfig);
        m_shooterMotorB.setControl(new Follower(Constants.IOMap.Shooter.kShooterMotorA, MotorAlignmentValue.Opposed));
    }

    public boolean shooterAtSpeed() {
        return Math.abs(Constants.Shooter.kMaxShooterSpeed - m_shooterMotorA.getVelocity().getValueAsDouble()) < Constants.Shooter.kShooterAtSpeedTolerance;
    }

    public Command stopShooter() {
        return new CommandBuilder(this)
            .onExecute(m_shooterMotorA::stopMotor)
            .isFinished(true);
    }

    public Command preheat() {
        return new CommandBuilder(this)
            .onExecute(() -> m_shooterMotorA.set(Constants.Shooter.kMaxShooterSpeed))
            .isFinished(this::shooterAtSpeed);
    }
    
    public Command manual_shooter(DoubleSupplier speed) {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_shooterMotorA.set(speed.getAsDouble());
            })
            .isFinished(() -> false);
    }
}