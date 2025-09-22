package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ManualElevatorControl extends Command {

    private final DoubleSupplier doubleSupplier;
    private final double speed = 1;

    /**
     * Manual setting of the elevator level by a controller
     *
     * @param doubleSupplier controller input from -1 to 1. Negative values lower and positive values raise the
     * setpoint.
     */
    public ManualElevatorControl(Elevator elevator, DoubleSupplier doubleSupplier) {
        this.doubleSupplier = doubleSupplier;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        Elevator.setTargetHeight(Elevator.getTargetHeight().plus(Inches.of(doubleSupplier.getAsDouble() * speed)));
    }
}
