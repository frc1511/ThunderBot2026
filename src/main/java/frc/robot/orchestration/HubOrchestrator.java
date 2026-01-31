package frc.robot.orchestration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class HubOrchestrator {
    private ShooterSubsystem shooter;
    public HubOrchestrator(ShooterSubsystem shooterSubsystem) { // TODO: other subsystems
        shooter = shooterSubsystem;
    }

    public Command shoot() {
        return shooter.shoot();
    }
}
