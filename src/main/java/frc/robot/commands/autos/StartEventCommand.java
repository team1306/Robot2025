package frc.robot.commands.autos;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class StartEventCommand extends Command {
    private final Command command;
    private final BooleanSupplier[] conditions;

    public StartEventCommand(Command command, BooleanSupplier... conditions) {
        this.command = command;
        this.conditions = conditions;
    }

    @Override
    public void execute() {
        if (Arrays.stream(conditions).allMatch(BooleanSupplier::getAsBoolean) && !command.isScheduled()) {
            command.schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) command.cancel();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }


}
