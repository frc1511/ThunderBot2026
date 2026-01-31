package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.CommandBuilder;
import frc.util.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX m_shooterMotor;

    public ShooterSubsystem() {
        m_shooterMotor = new TalonFX(Constants.IOMap.Shooter.shooterMotor);
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_shooterMotor.getConfigurator().apply(shooterConfig);
    }

    public boolean shooterAtSpeed() {
        return Math.abs(Constants.Shooter.kMaxShooterSpeed - m_shooterMotor.getVelocity().getValueAsDouble()) < Constants.Shooter.kShooterAtSpeedTolerance;
    }

    public Command stopShooter() {
        return new CommandBuilder(this)
            .onExecute(m_shooterMotor::stopMotor)
            .isFinished(true);
    }

    public Command preheat() {
        return new CommandBuilder(this)
            .onExecute(() -> m_shooterMotor.set(Constants.Shooter.kMaxShooterSpeed))
            .isFinished(this::shooterAtSpeed);
    }
}