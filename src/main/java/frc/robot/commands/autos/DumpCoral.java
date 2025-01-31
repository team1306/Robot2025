package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ConditionalWaitCommand;
import frc.robot.commands.arm.ArmSetpoint;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.elevator.MoveElevatorToSetpoint;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.commands.wrist.WristSetpoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class DumpCoral extends ParallelCommandGroup{
        public DumpCoral(Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
        addCommands(
            new MoveElevatorToSetpoint(elevator, ElevatorSetpoint.CORAL_L1),
            new ConditionalWaitCommand(() -> elevator.getCurrentHeight().gt(elevator.getTargetHeight().minus(Inches.of(4))))
            .andThen(
                new ParallelCommandGroup(
                    new MoveArmToSetpoint(arm, ArmSetpoint.CORAL_L1),
                    new MoveWristToSetpoint(wrist, WristSetpoint.HORIZONTAL)
            ))
            .andThen(
                new RunIntake(intake, () -> -1).withTimeout(Seconds.of(1))
            )
        );

    }
}
