package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public enum ElevatorSetpoints implements ElevatorSetpoint {

    CORAL_L1(Inches.of(18)),
    CORAL_L2(Inches.of(6)),
    CORAL_L3(Inches.of(18)),
    CORAL_L4(Inches.of(53.25)),

    CORAL_HOVER_L2(Inches.of(10)),
    CORAL_HOVER_L3(Inches.of(25)),

    CORAL_STATION(Inches.of(8)),
    BUMPER_CORAL_STATION(Inches.of(4)),
    GROUND_CORAL(Inches.of(3)),

    ALGAE_L2_REMOVE(Inches.of(19)),
    ALGAE_L3_REMOVE(Inches.of(27)),

    ALGAE_L2_REMOVE_END(Inches.of(24)),
    ALGAE_L3_REMOVE_END(Inches.of(32)),

    STOW(Inches.of(0));

    private final Distance height;

    /**
     * pre configured setpoints for the elevator
     *
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
