package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Utilities;

public class MoveElevatorToSetpoint extends Command {

    private final double TOLERANCE = 0.5; //inches

    private final Elevator elevator;
    private final Distance targetHeight;
    private final boolean finishWhenDone;

    private static int LAST_LEVEL;

    private int level;


    /**
     * Sets the elevator level
     * @param elevatorSetpoint setpoint for the elevator to go to
     */
    public MoveElevatorToSetpoint(Elevator elevator, ElevatorSetpoint elevatorSetpoint, boolean finishWhenDone) {
        this.targetHeight = elevatorSetpoint.getHeight();
        this.elevator = elevator;  

        this.finishWhenDone = finishWhenDone;

        this.level = elevatorSetpoint.getLevel();

        addRequirements(elevator);
    }

    public MoveElevatorToSetpoint(Elevator elevator, ElevatorSetpoint elevatorSetpoint) {
        this(elevator, elevatorSetpoint, true);
    }

    @Override
    public void execute() {
        elevator.setTargetHeight(targetHeight);
        MoveElevatorToSetpoint.LAST_LEVEL = level;
    }

    @Override
    public boolean isFinished() {
        return finishWhenDone && Utilities.isEqual(targetHeight.in(Inches), elevator.getCurrentHeight().in(Inches), TOLERANCE);
    }

    public static int getLastLevel() {
        return MoveElevatorToSetpoint.LAST_LEVEL;
    }
}
