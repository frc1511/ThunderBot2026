package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public Command shoot() {
        return new CommandBuilder(this)
            .onExecute(() -> m_shooterMotor.set(Constants.Shooter.kMaxShooterSpeed))
            .isFinished(true);
    }

    public Command stopShoot() {
        return new CommandBuilder(this)
            .onExecute(m_shooterMotor::stopMotor)
            .isFinished(true);
    }

    public Command turretToPosition(double targetPosition) {
        return new CommandBuilder(this) 
            .onExecute(() -> m_turretMotor.setControl(new PositionVoltage(targetPosition)))
            .isFinished(() -> m_turretMotor.getClosedLoopError().getValueAsDouble() < Constants.Shooter.kTurretTolerance);
    }

    public Command hoodToPosition(double targetPosition) {
        return new CommandBuilder(this) 
            .onExecute(() -> m_hoodMotor.setControl(new PositionVoltage(targetPosition)))
            .isFinished(() -> m_hoodMotor.getClosedLoopError().getValueAsDouble() < Constants.Shooter.kHoodTolerance);
    }  
}
