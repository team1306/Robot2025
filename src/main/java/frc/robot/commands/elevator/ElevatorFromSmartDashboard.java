package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.GetValue;

public class ElevatorFromSmartDashboard extends Command {

    private final Elevator elevator;
    @GetValue
    private double targetHeightInches;
    
    /**
     * Sets the elevator level
     * @param elevatorSetpoint setpoint for the elevator to go to
     */
    public ElevatorFromSmartDashboard(Elevator elevator) {
        this.elevator = elevator;
        DashboardHelpers.addUpdateClass(this);

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setTargetHeight(Inches.of(targetHeightInches));
    }
}
