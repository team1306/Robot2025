package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Utilities;

public class MoveElevatorToSetpoint extends Command {

    private final double TOLERANCE = 0.5; //inches

    private final Elevator elevator;
    private final Distance targetHeight;
    
    /**
     * Sets the elevator level
     * @param elevatorSetpoint setpoint for the elevator to go to
     */
    public MoveElevatorToSetpoint(Elevator elevator, ElevatorSetpoint elevatorSetpoint) {
        this.targetHeight = elevatorSetpoint.height;
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setTargetHeight(targetHeight);
    }

    @Override
    public boolean isFinished() {
        return Utilities.isEqual(targetHeight.in(Inches), elevator.getCurrentHeight().in(Inches), TOLERANCE);
    }
}
