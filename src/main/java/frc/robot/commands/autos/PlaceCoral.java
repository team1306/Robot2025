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

public class PlaceCoral extends ParallelCommandGroup {
    
    public PlaceCoral(Elevator elevator, Arm arm, Wrist wrist, int level) {

        if (level < 1 || level > 4) throw new IllegalArgumentException("Coral level must be between 1 and 4");

        ElevatorSetpoint elevatorSetpoint = ElevatorSetpoint.values()[level - 1];
        ArmSetpoint armSetpoint = ArmSetpoint.values()[level - 1];
        WristSetpoint wristSetpoint = WristSetpoint.values()[level - 1];
        

        addCommands(
            new MoveElevatorToSetpoint(elevator, elevatorSetpoint),
            new SequentialCommandGroup(
                new WaitCommand(0),
                new ParallelCommandGroup(
                    new MoveArmToSetpoint(arm, armSetpoint),
                    new MoveWristToSetpoint(wrist, wristSetpoint)
                )
            )
        );

    }
}
