package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.util.dashboardv3.entry.Config;
import frc.robot.util.dashboardv3.entry.Entry;
import frc.robot.util.dashboardv3.entry.EntryType;
import frc.robot.util.dashboardv3.networktables.mappings.UnitMappings;

public class ElevatorFromSmartDashboard extends Command {    
    @Entry(type = EntryType.Subscriber)
    @Config(UnitMappings.DistanceConfiguration.INCHES)
    private static Distance targetHeight = Inches.of(0);
    
    /**
     * Sets the elevator level
     * @param elevatorSetpoint setpoint for the elevator to go to
     */
    public ElevatorFromSmartDashboard(Elevator elevator) {
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        Elevator.setTargetHeight(targetHeight);
    }
}
