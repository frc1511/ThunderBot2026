package frc.robot.orchestration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.PivotSubsystem;
import frc.robot.subsystems.Storage.SpindexerSubsystem;
import frc.util.Constants;

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
        return pivot.down()
            .alongWith(intake.eat());
    }

    /**
     * Retract intake from ground and stop intaking FUEL
     */
    public Command excuseYourself() {
        return pivot.up()
            .alongWith(intake.stopEating());
    }

    public Command jostle() {
        return pivot.jostle()
            .raceWith(intake.eat());
    }

    public boolean isIntaking() {
        Command currentCommand = intake.getCurrentCommand();
        if (currentCommand == null) {
            return false;
        }

        return currentCommand.getName().equals(Constants.Hunger.Intake.intakeCommandName);
    }
}
