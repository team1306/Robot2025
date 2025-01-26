package frc.robot.commands.autos;

import edu.wpi.first.math.controller.ElevatorFeedforward;
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

public class Stow extends ParallelCommandGroup {
    
    public Stow (Elevator elevator, Arm arm, Wrist wrist) {
        addCommands(
            new MoveElevatorToSetpoint(elevator, ElevatorSetpoint.STOW),
            new MoveArmToSetpoint(arm, ArmSetpoint.STOW),
            new MoveWristToSetpoint(wrist, WristSetpoint.HORIZONTAL)
        );
    }
}
