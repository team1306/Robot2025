package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public enum ElevatorSetpoints implements ElevatorSetpoint {
    //Todo tune to find proper positions for elevator to drop coral
    CORAL_L1(Inches.of(18 - Constants.ELEVATOR_STARTING_HEIGHT)),
    CORAL_L2(Inches.of(31.875 - Constants.ELEVATOR_STARTING_HEIGHT)),
    CORAL_L3(Inches.of(47.625 - Constants.ELEVATOR_STARTING_HEIGHT)),
    CORAL_L4(Inches.of(72 - Constants.ELEVATOR_STARTING_HEIGHT)),
    
    CORAL_STATION(Inches.of(0)),

    STOW(Inches.of(Constants.ELEVATOR_STARTING_HEIGHT));

    private final Distance height;

    /**
     * pre configured setpoints for the elevator
     * @param height distance value
     */
    ElevatorSetpoints(Distance height) {
        this.height = height;
    }

    @Override
    public Distance getHeight() {
        return height;
    }

    @Override
    public int getLevel() {
        return (this.name().contains("CORAL_L")) ? Character.valueOf(this.name().charAt(7)) : 1;
    }
}