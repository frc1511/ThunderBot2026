package frc.robot.orchestration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.PivotSubsystem;
import frc.robot.subsystems.Storage.SpindexerSubsystem;
import frc.robot.subsystems.Storage.SpindexerSubsystem.SpinDuration;

/** Intake + Storage */
public class HungerOrchestrator {
    SpindexerSubsystem spindexer;
    IntakeSubsystem intake;
    PivotSubsystem pivot;

    public HungerOrchestrator(Robot robot) {
        spindexer = robot.spindexer;
        intake = robot.intake;
        pivot = robot.pivot;
    }

    /**
     * Put the intake down while intaking FUEL
     */
    public Command consume() {
        return pivot.pivotDown().alongWith(intake.eat());
    }

    /**
     * Retract intake from ground and stop intaking FUEL
     */
    public Command excuseYourself() {
        return pivot.pivotUp().alongWith(intake.stopEating());
    }

    /**
     * Intake the FUEL and spin the SPINDEXER
    */
    public Command feast() {
        return spindexer.spin(SpinDuration.INTAKE)
            .alongWith(consume())
            .withDeadline(Commands.none())
            .until(() -> true);
    }
}
