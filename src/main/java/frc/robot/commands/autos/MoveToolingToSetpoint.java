package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
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
    public MoveToolingToSetpoint(Elevator elevator, Arm arm, Wrist wrist, ElevatorSetpoint elevatorSetpoint, ArmSetpoint armSetpoint, WristSetpoint wristSetpoint, boolean overrideSafetyMode, boolean endWhenFinished) {
        if (overrideSafetyMode)
            addCommands(
                new MoveElevatorToSetpoint(elevator, elevatorSetpoint, endWhenFinished),
                new MoveArmToSetpoint(arm, armSetpoint, endWhenFinished),
                new MoveWristToSetpoint(wrist, wristSetpoint, endWhenFinished)
            );
        else 
            addCommands(
                new MoveElevatorToSetpoint(elevator, elevatorSetpoint, endWhenFinished),
                new StartEventCommand(
                    new MoveArmToSetpoint(arm, armSetpoint, endWhenFinished),
                    () -> !(Math.abs(wrist.getCurrentAngle().getDegrees()) > 30 && armSetpoint.getAngle().getDegrees() > 70) //go unless wrist is too rotated for how high the arm is trying to go
                ),
                new StartEventCommand( //only move wrist when arm is between threshold degrees/elevator is above threshold
                    new MoveWristToSetpoint(wrist, wristSetpoint, endWhenFinished),
                    () -> elevator.getCurrentHeight().in(Inches) > 10 || arm.getCurrentAngle().getDegrees() > -40,
                    () -> arm.getCurrentAngle().getDegrees() < 80
                )
            );
    }
    /**
     * Move elevator, arm, and wrist to setpoints in a parallel command group
     */
    public MoveToolingToSetpoint(Elevator elevator, Arm arm, Wrist wrist, ElevatorSetpoint elevatorSetpoint, ArmSetpoint armSetpoint, WristSetpoint wristSetpoint) {
        this(elevator, arm, wrist, elevatorSetpoint, armSetpoint, wristSetpoint, RobotContainer.isSafeMode(), false);
    }

    public MoveToolingToSetpoint(Elevator elevator, Arm arm, Wrist wrist, ElevatorSetpoint elevatorSetpoint, ArmSetpoint armSetpoint, WristSetpoint wristSetpoint, boolean endWhenFinished) {
        this(elevator, arm, wrist, elevatorSetpoint, armSetpoint, wristSetpoint, RobotContainer.isSafeMode(), endWhenFinished);
    }
}
