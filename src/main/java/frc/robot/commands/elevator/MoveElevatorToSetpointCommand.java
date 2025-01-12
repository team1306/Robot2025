package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveElevatorToSetpointCommand extends Command {
    private Elevator elevator;
    
    private double setpointInches;

    public MoveElevatorToSetpointCommand(Elevator elevator, double setpointInches) {
        this.setpointInches = setpointInches;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    public MoveElevatorToSetpointCommand(Elevator elevator, ElevatorSetpoints elevatorSetpoint) {
        this.setpointInches = elevatorSetpoint.height;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setTargetHeight(Inches.of(setpointInches));
    }

    @Override
    public boolean isFinished() {
        return elevator.getCurrentHeight().in(Inches) == setpointInches;
    }
}
