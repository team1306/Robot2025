package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmSetpoint;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.elevator.MoveElevatorToSetpoint;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.commands.wrist.WristSetpoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class PlaceCoralL4 extends ParallelCommandGroup {
    
    public PlaceCoralL4(Elevator elevator, Arm arm, Wrist wrist) {

        addCommands(
            new MoveElevatorToSetpoint(elevator, ElevatorSetpoint.CORAL_L4),
            new SequentialCommandGroup(
                new WaitCommand(0),
                new ParallelCommandGroup(
                    new MoveArmToSetpoint(arm, ArmSetpoint.CORAL_L4),
                    new MoveWristToSetpoint(wrist, WristSetpoint.VERTICAL)
                )
            )
        );
    }
}
