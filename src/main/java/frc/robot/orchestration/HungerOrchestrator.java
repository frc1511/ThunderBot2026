package frc.robot.orchestration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Storage.SpindexerSubsystem;
import frc.robot.subsystems.Storage.SpindexerSubsystem.SpinDuration;
import frc.util.CommandBuilder;

// Intake + Storage
public class HungerOrchestrator {
    SpindexerSubsystem spindexer;
    IntakeSubsystem intake;

    public HungerOrchestrator(Robot robot) {
        spindexer = robot.spindexer;
        intake = robot.intake;
    }

    /**
     * Intake the FUEL and spin the SPINDEXER
    */
    // public Command feast() {
    //     new Command().
    //     return spindexer.spin(SpinDuration.INTAKE).alongWith(intake.consume());
    // }
}
