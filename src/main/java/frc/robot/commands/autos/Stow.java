package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ArmSetpoints;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoints;
import frc.robot.commands.elevator.MoveElevatorToSetpoint;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.commands.wrist.WristSetpoints;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class Stow extends ParallelCommandGroup {
    
    /**
     * returns the elevator, arm, and wrist to stow position.
     */
    public Stow (Elevator elevator, Arm arm, Wrist wrist) {
        addCommands(
            new MoveElevatorToSetpoint(elevator, ElevatorSetpoints.STOW),
            new MoveArmToSetpoint(arm, ArmSetpoints.STOW),
            new MoveWristToSetpoint(wrist, WristSetpoints.HORIZONTAL)
        );
    }
}
