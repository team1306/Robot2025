package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Inches;

import badgerlog.entry.Config;
import badgerlog.entry.Entry;
import badgerlog.entry.EntryType;
import badgerlog.networktables.mappings.UnitMappings;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorFromSmartDashboard extends Command {    
    @Entry(type = EntryType.Subscriber)
    @Config(UnitMappings.DistanceConfiguration.INCHES)
    private static Distance targetHeight = Inches.of(0);
    
    /**
     * Sets the elevator level
     */
    public ElevatorFromSmartDashboard(Elevator elevator) {
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        Elevator.setTargetHeight(targetHeight);
    }
}
