package frc.robot.orchestration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Cannon.HoodSubsystem;
import frc.robot.subsystems.Cannon.ShooterSubsystem;
import frc.robot.subsystems.Storage.SpindexerSubsystem;
import frc.util.Constants;
import frc.robot.subsystems.Storage.KickerSubsystem;

public class FiringOrchestrator {
    ShooterSubsystem shooter;
    KickerSubsystem kicker;
    SpindexerSubsystem spindexer;
    HoodSubsystem hood;

    public FiringOrchestrator(Robot robot) {
        shooter = robot.shooter;
        kicker = robot.kicker;
        spindexer = robot.spindexer;
        hood = robot.hood;
    }

    public Command fire() {
        return new ConditionalCommand(
            // If not feeding (normal hub shot)
            new ParallelCommandGroup(
                shooter.preheat(),
                hood.toOptimalPosition()
            ).andThen(
                new ParallelCommandGroup(
                    shooter.holdSpeedForShoot(),
                    kicker.run(),
                    spindexer.spin(Constants.Storage.Spindexer.Duration.FOREVER),
                    hood.toOptimalPosition()
                )
            )
            .withName("fire"),

            // If feeding (across field)
            shooter.preheat()
                .andThen(
                    new ParallelCommandGroup(
                        shooter.holdSpeedForShoot(),
                        kicker.run(),
                        spindexer.spin(Constants.Storage.Spindexer.Duration.FOREVER)
                    ))
            .withName("feed"),

            // The condition for the above commands (true == top, false == bottom)
            () -> hood.getTargetPosition() != Constants.Hood.Position.FEED);
    }

    public Command fireThenStop() {
        return new ParallelCommandGroup(
                shooter.preheat(),
                hood.toOptimalPosition()
            ).andThen(
                new ParallelCommandGroup(
                    shooter.holdSpeedForShoot(),
                    kicker.run(),
                    spindexer.spin(Constants.Storage.Spindexer.Duration.FOREVER),
                    hood.toOptimalPosition()
                ));
    }

    public Command halt() {
        return new ParallelCommandGroup(
            shooter.halt(),
            kicker.halt(),
            spindexer.halt()
        ).withDeadline(new InstantCommand());
    }
}
