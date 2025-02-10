package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashMap;
import java.util.function.Supplier;

/**
 * Constructs a command to wrap a choice without using multiple .onlyIf() decorators on commands and running them together
 * @param <T> the type of the selector
 */
public class ConditionalCommandChooser<T> extends Command {
    
    private final HashMap<T, Command> commandMap;
    private final Supplier<T> supplier;
    
    public ConditionalCommandChooser(HashMap<T, Command> commands, Supplier<T> supplier) {
        this.commandMap = commands;
        this.supplier = supplier;
    }
    
    @Override
    public void initialize() {
        commandMap.get(supplier.get()).schedule();
    }
}
