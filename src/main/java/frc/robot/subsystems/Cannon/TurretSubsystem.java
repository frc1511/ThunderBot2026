package frc.robot.subsystems.Cannon;

import java.util.function.Supplier;

// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.util.CommandBuilder;
// import frc.util.Constants;




// No turret on the robot, for future use




public class TurretSubsystem extends SubsystemBase {
    // private TalonFX m_turretMotor;
    
    public TurretSubsystem() {
        // m_turretMotor = new TalonFX(Constants.IOMap.Shooter.kturretMotor);
        // TalonFXConfiguration turretConfig = new TalonFXConfiguration(); 
        // turretConfig.Slot0 = new Slot0Configs()
        //     .withKP(Constants.Shooter.TurretPID.kP).withKI(Constants.Shooter.TurretPID.kI).withKD(Constants.Shooter.TurretPID.kD);
        // m_turretMotor.getConfigurator().apply(turretConfig);
    }
        
    public Command toPosition(Supplier<Double> targetPosition) {
        return Commands.none();
        // return new CommandBuilder(this) 
        //     .onExecute(() -> m_turretMotor.setControl(new PositionVoltage(targetPosition.get())))
        //     .isFinished(this::turretAtPosition);
    }

    public boolean turretAtPosition() {
        return true;
        // return m_turretMotor.getClosedLoopError().getValueAsDouble() < Constants.Shooter.kTurretTolerance;
    }
}


