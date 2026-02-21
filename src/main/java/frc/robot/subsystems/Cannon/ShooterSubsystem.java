package frc.robot.subsystems.Cannon;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    private double m_targetSpeed = 0;

    public ShooterSubsystem() {
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfig.Slot0 = new Slot0Configs()
            .withKP(Constants.Shooter.ShooterPID.kP).withKI(Constants.Shooter.ShooterPID.kI).withKD(Constants.Shooter.ShooterPID.kD);
        shooterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
        
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

        return Math.abs(Helpers.RPStoRPM(m_primaryMotor.getVelocity().getValueAsDouble()) - m_targetSpeed) < Constants.Shooter.kShooterAtSpeedTolerance;
    }

    @Override
    public void periodic() {
        if (Broken.shooterFullyDisabled) return;

        SmartDashboard.putNumber("shooter_rpm", Helpers.RPStoRPM(m_primaryMotor.getVelocity().getValueAsDouble()));
        SmartDashboard.putNumber("shooter_target_rpm", Helpers.RPStoRPM(m_primaryMotor.getClosedLoopReference().getValueAsDouble()));
        SmartDashboard.putNumber("shooter_output_V", m_primaryMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("shooter_err", Math.abs(Helpers.RPStoRPM(m_primaryMotor.getVelocity().getValueAsDouble()) - m_targetSpeed));
        SmartDashboard.putBoolean("shooter_atSpeed", shooterAtSpeed());
    }

    public Command halt() {
        if (Broken.hoodDisabled) return new InstantCommand(()->{}, this);

        return new CommandBuilder(this)
            .onExecute(m_primaryMotor::stopMotor);
    }

    private void runAtSpeed(double speedRPM) {
        m_targetSpeed = speedRPM;
        m_primaryMotor.setControl(new VelocityVoltage(Helpers.RPMtoRPS(m_targetSpeed)));
    }

    public Command runAtCustomSpeed(DoubleSupplier speedRPM) {
        if (Broken.shooterFullyDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> runAtSpeed(speedRPM.getAsDouble()))
            // .isFinished(this::shooterAtSpeed)
            .onEnd(this::halt);
    }

    public Command preheat() {
        if (Broken.shooterFullyDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> runAtSpeed(Constants.Shooter.kTargetShooterRPM))
            .isFinished(this::shooterAtSpeed)
            .onEnd(this::halt);
    }

    public Command holdSpeedForShoot() {
        if (Broken.shooterFullyDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() -> runAtSpeed(Constants.Shooter.kTargetShooterRPM))
            .onEnd(this::halt);
    }

    public Command manual_shooter(DoubleSupplier speed) {
        if (Broken.shooterFullyDisabled) return Commands.none();

        return new CommandBuilder(this)
            .onExecute(() ->
                runAtSpeed(speed.getAsDouble() * Helpers.RPMtoRPS(Constants.Shooter.kTargetShooterRPM))
            )
            .isFinished(() -> false);
    }

    public Status status() {
        if (Broken.shooterFullyDisabled) return Status.DISABLED;
        if (!Helpers.onCANChain(m_primaryMotor)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_primaryMotor)) return Status.ACTIVE;
        return Status.IDLE;
    }
}