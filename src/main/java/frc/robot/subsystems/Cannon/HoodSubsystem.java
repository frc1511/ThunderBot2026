package frc.robot.subsystems.Cannon;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.CommandBuilder;
import frc.util.Constants;

public class HoodSubsystem extends SubsystemBase {
    private TalonFX m_hoodMotor;

    public HoodSubsystem() {
        m_hoodMotor = new TalonFX(Constants.IOMap.Shooter.kHoodMotor);
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration(); 
        hoodConfig.Slot0 = new Slot0Configs()
            .withKP(Constants.Shooter.HoodPID.kP).withKI(Constants.Shooter.HoodPID.kI).withKD(Constants.Shooter.HoodPID.kD);
        m_hoodMotor.getConfigurator().apply(hoodConfig);
    }

    public boolean hoodAtPosition() {
        return m_hoodMotor.getClosedLoopError().getValueAsDouble() < Constants.Shooter.kHoodTolerance;
    }

    public Command toPosition(Supplier<Double> targetPosition) {
        return new CommandBuilder(this) 
            .onExecute(() -> m_hoodMotor.setControl(new PositionVoltage(targetPosition.get())))
            .isFinished(this::hoodAtPosition);
    }

    public Command manual_hood(DoubleSupplier speed) {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_hoodMotor.set(speed.getAsDouble());
            });
    }
}