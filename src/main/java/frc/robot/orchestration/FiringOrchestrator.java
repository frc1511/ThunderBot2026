package frc.robot.orchestration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Cannon.ShooterSubsystem;
import frc.robot.subsystems.Storage.SpindexerSubsystem;
import frc.util.Constants;
import frc.robot.subsystems.Storage.KickerSubsystem;

public class FiringOrchestrator {
    ShooterSubsystem shooter;
    KickerSubsystem kicker;
    SpindexerSubsystem spindexer;

    public FiringOrchestrator(Robot robot) {
        shooter = robot.shooter;
        kicker = robot.kicker;
        spindexer = robot.spindexer;
    }

    public Command fire() {
        return shooter.preheat()
            .alongWith(
                new ParallelCommandGroup(
                    kicker.run(), 
                    spindexer.spin(Constants.Storage.Spindexer.Duration.PARTIAL_BAY.get())
                )
                .onlyIf(shooter::shooterAtSpeed));
    }
}
