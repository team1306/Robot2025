package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmSetpoints;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoints;
import frc.robot.commands.elevator.MoveElevatorToSetpoint;
import frc.robot.commands.intake.SpitCoral;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.commands.wrist.WristSetpoints;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class DumpCoral extends SequentialCommandGroup{

    public DumpCoral(Elevator elevator, Intake intake, Arm arm, Wrist wrist){
        addCommands(
            new ParallelCommandGroup(
                new MoveElevatorToSetpoint(elevator, ElevatorSetpoints.CORAL_L1),
                new MoveArmToSetpoint(arm, ArmSetpoints.CORAL_L1),
                new MoveWristToSetpoint(wrist, WristSetpoints.HORIZONTAL)
        ).andThen(new SpitCoral(intake)));
    }
}
