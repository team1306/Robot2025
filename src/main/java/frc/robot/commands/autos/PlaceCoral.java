package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ArmSetpoint;
import frc.robot.commands.arm.ArmSetpoints;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoints;
import frc.robot.commands.wrist.WristSetpoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class PlaceCoral extends ParallelCommandGroup {

    /**
     * Places coral on L2, L3 or L4.
     */
    public PlaceCoral(Elevator elevator, Arm arm, Wrist wrist, int level, WristSetpoint wristSetpoint) {
        if (level != 2 && level != 3 && level != 4) throw new RuntimeException("Level must be 2, 3, or 4");

        ElevatorSetpoint elevatorSetpoint = switch (level) {
            case 3 -> ElevatorSetpoints.CORAL_HOVER_L3;
            case 4 -> ElevatorSetpoints.CORAL_L4;

            default -> ElevatorSetpoints.CORAL_HOVER_L2;
        };

        ArmSetpoint hoverSetpoint = switch (level) {
            case 3 -> ArmSetpoints.HOVER_L3;
            case 4 -> ArmSetpoints.HOVER_L4;

            default -> ArmSetpoints.HOVER_L2;
        };

        addCommands(
                new MoveToolingToSetpoint(elevator, arm, wrist, elevatorSetpoint, hoverSetpoint, wristSetpoint, true)
        );

    }
}
