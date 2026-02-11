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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Broken;
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
        shooterConfig.Slot0 = new Slot0Configs()
            .withKP(Constants.Shooter.ShooterPID.kP).withKI(Constants.Shooter.ShooterPID.kI).withKD(Constants.Shooter.ShooterPID.kD);
        m_shooterMotorA.getConfigurator().apply(shooterConfig);
        m_shooterMotorB.getConfigurator().apply(shooterConfig);
        m_shooterMotorB.setControl(new Follower(Constants.IOMap.Shooter.kShooterMotorA, MotorAlignmentValue.Opposed));
    }

    public boolean shooterAtSpeed() {
        return m_shooterMotorA.getClosedLoopError().getValueAsDouble() < Constants.Shooter.kShooterAtSpeedTolerance;
        // return Math.abs(Constants.Shooter.kMaxShooterSpeed - m_shooterMotorA.getVelocity().getValueAsDouble()) < Constants.Shooter.kShooterAtSpeedTolerance;
    }

    public void periodic() {
        SmartDashboard.putNumber("Actual Output", m_shooterMotorA.getVelocity().getValueAsDouble() * 60);
    }

    public Command stopShooter() {
        return new CommandBuilder(this)
            .onExecute(m_shooterMotorA::stopMotor)
            .isFinished(true);
    }

    public Command preheat() {
        return new CommandBuilder(this)
            .onExecute(() -> m_shooterMotorA.setControl(new VelocityVoltage(Constants.Shooter.kTargetShooterRPM / 60)))
            .isFinished(this::shooterAtSpeed);
    }

    public Command manual_shooter(DoubleSupplier speed) {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_shooterMotorA.setControl(new VelocityVoltage(speed.getAsDouble() * Constants.Shooter.kTargetShooterRPM / 60));
            })
            .isFinished(() -> false);
    }

    public Command shoot() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                if (Broken.shooterFull) return;
                m_shooterMotorA.set(Constants.Shooter.kMaxShooterSpeed);
            })
            .isFinished(true);
    }
}