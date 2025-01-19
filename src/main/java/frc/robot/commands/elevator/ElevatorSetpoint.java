package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public enum ElevatorSetpoint {
    //DO NOT ADD ANY IN FRONT OF CORAL
    CORAL_L1(Inches.of(18 - Constants.ELEVATOR_STARTING_HEIGHT)),
    CORAL_L2(Inches.of(31.875 - Constants.ELEVATOR_STARTING_HEIGHT)),
    CORAL_L3(Inches.of(47.625 - Constants.ELEVATOR_STARTING_HEIGHT)),
    CORAL_L4(Inches.of(72 - Constants.ELEVATOR_STARTING_HEIGHT)),
    
    CORAL_STATION(Inches.of(0));

    public final Distance height;

    private ElevatorSetpoint(Distance height) {
        this.height = height;
    }
}