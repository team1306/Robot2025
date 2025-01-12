package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ManualElevatorControl extends Command {
    
    private Elevator elevator;
    private DoubleSupplier doubleSupplier; //this is the controller input

    public ManualElevatorControl(Elevator elevator, DoubleSupplier doubleSupplier) {
        this.elevator = elevator;
        this.doubleSupplier = doubleSupplier;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        final double SPEED = 1; //some random speed factor

        elevator.setTargetHeight(elevator.getTargetHeight().plus(Inches.of(doubleSupplier.getAsDouble() * SPEED)));
    }
}
