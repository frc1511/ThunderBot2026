package frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Alert;
import frc.util.CommandBuilder;
import frc.util.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX m_shooterMotor;
    private TalonFX m_turretMotor;
    private TalonFX m_hoodMotor;

    public ShooterSubsystem() {
        m_shooterMotor = new TalonFX(Constants.IOMap.Shooter.shooterMotor);
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_shooterMotor.getConfigurator().apply(shooterConfig);

        m_turretMotor = new TalonFX(Constants.IOMap.Shooter.turretMotor);
        TalonFXConfiguration turretConfig = new TalonFXConfiguration(); 
        turretConfig.Slot0 = new Slot0Configs()
            .withKP(Constants.Shooter.TurretPID.kP).withKI(Constants.Shooter.TurretPID.kI).withKD(Constants.Shooter.TurretPID.kD);
        m_turretMotor.getConfigurator().apply(turretConfig);

        m_hoodMotor = new TalonFX(Constants.IOMap.Shooter.hoodMotor);
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration(); 
        hoodConfig.Slot0 = new Slot0Configs()
            .withKP(Constants.Shooter.HoodPID.kP).withKI(Constants.Shooter.HoodPID.kI).withKD(Constants.Shooter.HoodPID.kD);
        m_hoodMotor.getConfigurator().apply(hoodConfig);
    }

    public boolean shooterAtSpeed() {
        return Math.abs(Constants.Shooter.kMaxShooterSpeed - m_turretMotor.getVelocity().getValueAsDouble()) < Constants.Shooter.kShooterAtSpeedTolerance;
    }

    public boolean turretAtPosition() {
        return m_turretMotor.getClosedLoopError().getValueAsDouble() < Constants.Shooter.kTurretTolerance;
    }

    public boolean hoodAtPosition() {
        return m_hoodMotor.getClosedLoopError().getValueAsDouble() < Constants.Shooter.kHoodTolerance;
    }

    public Command preheat() {
        return new CommandBuilder(this)
            .onExecute(() -> m_shooterMotor.set(Constants.Shooter.kMaxShooterSpeed))
            .isFinished(this::shooterAtSpeed);
    }

    public Command stopShooter() {
        return new CommandBuilder(this)
            .onExecute(m_shooterMotor::stopMotor)
            .isFinished(true);
    }

    public Command rapidFire() {
        return new CommandBuilder(this)
            .onExecute(() -> Alert.error("This is where we beam a command to storage, but its not implemented yet, may have to move to a state machine"));
    }

    public Command turretToPosition(Supplier<Double> targetPosition) {
        return new CommandBuilder(this) 
            .onExecute(() -> m_turretMotor.setControl(new PositionVoltage(targetPosition.get())))
            .isFinished(this::turretAtPosition);
    }

    public Command hoodToPosition(Supplier<Double> targetPosition) {
        return new CommandBuilder(this) 
            .onExecute(() -> m_hoodMotor.setControl(new PositionVoltage(targetPosition.get())))
            .isFinished(this::hoodAtPosition);
    }

    public Command autoFire(Supplier<Double> optimalTurretPosition, Supplier<Double> optimalHoodPosition) {
        return turretToPosition(optimalTurretPosition)
            .alongWith(hoodToPosition(optimalHoodPosition))
            .alongWith(preheat())
            .andThen(rapidFire());
    }
}
