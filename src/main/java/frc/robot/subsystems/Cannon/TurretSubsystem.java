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
import frc.util.Constants.Status;
import frc.util.Helpers;


public class TurretSubsystem extends SubsystemBase {
    private TalonFX m_motor;
    
    public TurretSubsystem() {
        TalonFXConfiguration turretConfig = new TalonFXConfiguration(); 
        turretConfig.Slot0 = new Slot0Configs()
            .withKP(Constants.Shooter.TurretPID.kP).withKI(Constants.Shooter.TurretPID.kI).withKD(Constants.Shooter.TurretPID.kD);

        if (!Broken.turretDisable) {
            m_motor = new TalonFX(Constants.IOMap.Shooter.kturretMotor);
            m_motor.getConfigurator().apply(turretConfig);
        } else {
            m_motor = null;
        }
    }
        
    public Command toPosition(Supplier<Double> targetPosition) {
        if (Broken.turretDisable) return Commands.none();

        return new CommandBuilder(this) 
            .onExecute(() -> m_motor.setControl(new PositionVoltage(targetPosition.get())))
            .isFinished(this::turretAtPosition);
    }

    public boolean turretAtPosition() {
        if (Broken.turretDisable) return true;
        
        return m_motor.getClosedLoopError().getValueAsDouble() < Constants.Shooter.kTurretTolerance;
    }

    public Status status() {
        if (Broken.turretDisable) return Status.DISABLED;
        if (!Helpers.onCANChain(m_motor)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_motor)) return Status.ACTIVE;
        return Status.IDLE;
    }
}


