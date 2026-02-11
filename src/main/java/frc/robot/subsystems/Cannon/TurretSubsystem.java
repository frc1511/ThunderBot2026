package frc.robot.subsystems.Cannon;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;


public class TurretSubsystem extends SubsystemBase {
    private TalonFX m_turretMotor;
    
    public TurretSubsystem() {
        TalonFXConfiguration turretConfig = new TalonFXConfiguration(); 
        turretConfig.Slot0 = new Slot0Configs()
            .withKP(Constants.Shooter.TurretPID.kP).withKI(Constants.Shooter.TurretPID.kI).withKD(Constants.Shooter.TurretPID.kD);

        if (!Broken.turret) {
            m_turretMotor = new TalonFX(Constants.IOMap.Shooter.kturretMotor);
            m_turretMotor.getConfigurator().apply(turretConfig);
        } else {
            m_turretMotor = null;
        }
    }
        
    public Command toPosition(Supplier<Double> targetPosition) {
        if (Broken.turret) return Commands.none();

        return new CommandBuilder(this) 
            .onExecute(() -> m_turretMotor.setControl(new PositionVoltage(targetPosition.get())))
            .isFinished(this::turretAtPosition);
    }

    public boolean turretAtPosition() {
        if (Broken.turret) return true;
        
        return m_turretMotor.getClosedLoopError().getValueAsDouble() < Constants.Shooter.kTurretTolerance;
    }
}


