package frc.robot.commands.elevator;

import frc.robot.Constants;

public enum ElevatorSetpoints {
    
    CORAL_L1(18 - Constants.ELEVATOR_STARTING_HEIGHT),
    CORAL_L2(31.875 - Constants.ELEVATOR_STARTING_HEIGHT),
    CORAL_L3(47.625 - Constants.ELEVATOR_STARTING_HEIGHT),
    CORAL_L4(72 - Constants.ELEVATOR_STARTING_HEIGHT);

    

    public final double height; //inches

    private ElevatorSetpoints(double inches) {
        this.height = inches;
    }
}