package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ArmSetpoint;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.elevator.MoveElevatorToSetpoint;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.commands.wrist.WristSetpoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class MoveToolingToSetpoint extends ParallelCommandGroup {
    /**
     * Move elevator, arm, and wrist to setpoints in a parallel command group
     */
    public MoveToolingToSetpoint(Elevator elevator, Arm arm, Wrist wrist, ElevatorSetpoint elevatorSetpoint, ArmSetpoint armSetpoint, WristSetpoint wristSetpoint) {
        addCommands(
            new MoveElevatorToSetpoint(elevator, elevatorSetpoint),
            new MoveArmToSetpoint(arm, armSetpoint),
            new MoveWristToSetpoint(wrist, wristSetpoint)
        );
    }


}
