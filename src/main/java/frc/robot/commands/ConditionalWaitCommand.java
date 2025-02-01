package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ConditionalWaitCommand extends Command {
    
    private BooleanSupplier condition;
    
    /**
     * Sets the angle of the wrist
     */
    public ConditionalWaitCommand (BooleanSupplier condition) {
        this.condition = condition;
    }

    @Override 
    public boolean isFinished() {
        return condition.getAsBoolean();
    }
}
