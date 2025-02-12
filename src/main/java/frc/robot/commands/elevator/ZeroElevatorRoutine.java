package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Inches;

public class ZeroElevatorRoutine extends Command {
    
    private final Elevator elevator;
    
    public ZeroElevatorRoutine(Elevator elevator) {
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        elevator.setTargetHeight(Inches.of(0));
    }
    
    @Override
    public void execute() {
        if(elevator.atSetpoint()){
            elevator.setTargetHeight(elevator.getTargetHeight().minus(Inches.of(1)));
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        elevator.setTargetHeight(ElevatorSetpoints.STOW.getHeight());
    }

    @Override
    public boolean isFinished() {
        return elevator.getLimitSwitch();
    }
}
