package frc.robot.commands;

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
import lombok.Getter;

public class MoveToolingToSetpoint extends ParallelCommandGroup {
    @Getter
    private static int lastLevel;
    public MoveToolingToSetpoint(
        Arm arm, Elevator elevator, Wrist wrist,
        ElevatorSetpoint elevatorSetpoint, ArmSetpoint armSetpoint, WristSetpoint wristSetpoint) {
        addCommands(/*
            new InstantCommand(() -> lastLevel = switch (elevatorSetpoint) {
                case ElevatorSetpoints.CORAL_L1 -> 1;
                default -> 1;
            }),
            */
            new MoveElevatorToSetpoint(elevator, elevatorSetpoint, false),
            new MoveArmToSetpoint(arm, armSetpoint, false),
            new MoveWristToSetpoint(wrist, wristSetpoint, false)
        );
    }


}
