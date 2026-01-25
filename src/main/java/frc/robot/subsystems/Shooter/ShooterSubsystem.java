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
    private TalonFXConfiguration configuration;
    private TalonFXConfiguration turretConfiguration;
    private TalonFXConfiguration hoodConfiguration;

    public ShooterSubsystem() {
        m_shooterMotor = new TalonFX(Constants.IOMap.Shooter.shooterMotor);
        configuration = new TalonFXConfiguration();
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_shooterMotor.getConfigurator().apply(configuration);

        m_turretMotor = new TalonFX(Constants.IOMap.Shooter.turretMotor);
        turretConfiguration = new TalonFXConfiguration(); 
        turretConfiguration.Slot0 = new Slot0Configs()
            .withKP(0).withKI(0).withKD(0); // just placeholder values
        m_turretMotor.getConfigurator().apply(turretConfiguration);
    
        m_hoodMotor = new TalonFX(Constants.IOMap.Shooter.hoodMotor);
        hoodConfiguration = new TalonFXConfiguration(); 
        hoodConfiguration.Slot0 = new Slot0Configs()
            .withKP(0).withKI(0).withKD(0); // just placeholder values
        m_hoodMotor.getConfigurator().apply(hoodConfiguration);
    }

    public Command shooterRun() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_shooterMotor.set(.1);
            });
    }

    public Command shooterStop() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_shooterMotor.stopMotor();
            })
            .isFinished(() -> true);
    }

    public Command turretToPosition(double targetPosition) {
        return new CommandBuilder(this) 
            .onExecute(() -> {
                m_turretMotor.setControl(new PositionVoltage(targetPosition));
            })
            .isFinished(() ->{
                return m_turretMotor.getClosedLoopError().getValueAsDouble() < .5d;
            });
    }

    public Command hoodToPosition(double targetPosition) {
        return new CommandBuilder(this) 
            .onExecute(() -> {
                m_hoodMotor.setControl(new PositionVoltage(targetPosition));
            })
            .isFinished(() ->{
                return m_hoodMotor.getClosedLoopError().getValueAsDouble() < .5d;
            });
    }  
}
