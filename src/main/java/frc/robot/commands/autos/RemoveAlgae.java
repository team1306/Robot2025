package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmSetpoint;
import frc.robot.commands.arm.ArmSetpoints;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoints;
import frc.robot.commands.wrist.WristSetpoints;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class RemoveAlgae extends SequentialCommandGroup {
    public RemoveAlgae(Elevator elevator, Arm arm, Wrist wrist, int level) {
        if (level != 2 && level != 3) throw new RuntimeException("Level must be 2 or 3");

        ElevatorSetpoint elevatorSetpoint = level == 2 ? ElevatorSetpoints.ALGAE_L2_REMOVE_END : ElevatorSetpoints.ALGAE_L3_REMOVE_END;

        ArmSetpoint armSetpoint = level == 2 ? ArmSetpoints.ALGAE_L2_REMOVE : ArmSetpoints.ALGAE_L3_REMOVE;

        addCommands(new MoveToolingToSetpoint(elevator, arm, wrist, elevatorSetpoint, armSetpoint, WristSetpoints.HORIZONTAL, true));
    }
}
