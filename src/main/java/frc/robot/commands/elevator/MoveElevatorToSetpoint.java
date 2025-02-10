package frc.robot.commands.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveElevatorToSetpoint extends Command {

    private final Elevator elevator;
    private final Distance targetHeight;
    private final boolean finishWhenDone;

    /**
     * Sets the elevator level
     * @param elevatorSetpoint setpoint for the elevator to go to
     */
    public MoveElevatorToSetpoint(Elevator elevator, ElevatorSetpoint elevatorSetpoint, boolean finishWhenDone) {
        this.targetHeight = elevatorSetpoint.getHeight();
        this.elevator = elevator;  

        this.finishWhenDone = finishWhenDone;
        
        addRequirements(elevator);
    }

    public MoveElevatorToSetpoint(Elevator elevator, ElevatorSetpoint elevatorSetpoint) {
        this(elevator, elevatorSetpoint, true);
    }

    @Override
    public void execute() {
        elevator.setTargetHeight(targetHeight);
    }

    @Override
    public boolean isFinished() {
        return finishWhenDone && elevator.atSetpoint();
    }
}
