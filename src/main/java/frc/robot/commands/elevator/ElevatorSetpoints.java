package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public enum ElevatorSetpoints implements ElevatorSetpoint {

    CORAL_L1(Inches.of(2)),
    CORAL_L2(Inches.of(17)),
    CORAL_L3(Inches.of(34)),
    CORAL_L4(Inches.of(53)),
    
    CORAL_STATION(Inches.of(2)),

    STOW(Inches.of(2));

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
}