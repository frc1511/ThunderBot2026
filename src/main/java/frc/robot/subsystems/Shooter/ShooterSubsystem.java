package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.CommandBuilder;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX motor;
    private TalonFXConfiguration configuration;

    public ShooterSubsystem() {
        configuration = new TalonFXConfiguration();
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motor = new TalonFX(20);
        motor.getConfigurator().apply(configuration);
    }
    public Command runMotor() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                motor.set(.1);
            });
    }

    public Command stopMotor() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                motor.stopMotor();
            })
            .isFinished(() -> true);
    }
}
