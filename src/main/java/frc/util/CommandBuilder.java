package frc.util;

import java.util.ConcurrentModificationException;
import java.util.function.BooleanSupplier;
import edu.wpi.first.util.function.BooleanConsumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/*
 * Based on GRR 340's CommandBuilder
 */
public class CommandBuilder extends Command {
    private Runnable onInitialize = () -> {};
    private Runnable onExecute = () -> {};
    private BooleanConsumer onEnd = interrupted -> {};
    private BooleanSupplier isFinished = () -> false;

    public CommandBuilder(String name, Subsystem... requirements) {
        this(requirements);
        setName(name);
    }

    public CommandBuilder(Subsystem... requirements) {
        addRequirements(requirements);
    }

    public CommandBuilder onInitialize(Runnable onInitialize) {
        if (this.isScheduled()) throw new ConcurrentModificationException(
            "Can't change a command's methods while scheduled"
        );
        this.onInitialize = onInitialize;
        return this;
    }

    public CommandBuilder onExecute(Runnable onExecute) {
        if (this.isScheduled()) throw new ConcurrentModificationException(
            "Can't change a command's methods while scheduled"
        );
        this.onExecute = onExecute;
        return this;
    }

    public CommandBuilder onEnd(Runnable onEnd) {
        return onEnd(interrupted -> onEnd.run());
    }

    public CommandBuilder onEnd(BooleanConsumer onEnd) {
        if (this.isScheduled()) throw new ConcurrentModificationException(
            "Can't change a command's methods while scheduled"
        );
        this.onEnd = onEnd;
        return this;
    }

    public CommandBuilder isFinished(boolean isFinished) {
        return isFinished(() -> isFinished);
    }

    public CommandBuilder isFinished(BooleanSupplier isFinished) {
        if (this.isScheduled()) throw new ConcurrentModificationException(
            "Can't change a command's methods while scheduled"
        );
        this.isFinished = isFinished;
        return this;
    }

    @Override
    public void initialize() {
        onInitialize.run();
    }

    @Override
    public void execute() {
        onExecute.run();
    }

    @Override
    public void end(boolean interrupted) {
        onEnd.accept(interrupted);
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }
}
