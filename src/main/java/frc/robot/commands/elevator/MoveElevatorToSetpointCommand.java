package frc.robot.commands.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveElevatorToSetpointCommand extends Command {
    private Elevator elevator;
    
    private Distance targetHeight;
    
    public MoveElevatorToSetpointCommand(Elevator elevator, ElevatorSetpoints elevatorSetpoint) {
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
        return elevator.getCurrentHeight() == targetHeight;
    }
}
