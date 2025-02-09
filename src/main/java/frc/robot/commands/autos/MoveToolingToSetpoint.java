package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Inches;

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
    public MoveToolingToSetpoint(Elevator elevator, Arm arm, Wrist wrist, ElevatorSetpoint elevatorSetpoint, ArmSetpoint armSetpoint, WristSetpoint wristSetpoint, boolean overrideSafetyMode) {
        if (overrideSafetyMode)
            addCommands(
                new MoveElevatorToSetpoint(elevator, elevatorSetpoint),
                new MoveArmToSetpoint(arm, armSetpoint),
                new MoveWristToSetpoint(wrist, wristSetpoint)
            );
        else 
            addCommands(
                new MoveElevatorToSetpoint(elevator, elevatorSetpoint),
                new StartEventCommand(
                    new MoveArmToSetpoint(arm, armSetpoint),
                    () -> !(Math.abs(wrist.getCurrentAngle().getDegrees()) > 30 && armSetpoint.getAngle().getDegrees() > 70) //go unless wrist is too rotated for how high the arm is trying to go
                ),
                new StartEventCommand( //only move wrist when arm is between threshold degrees/elevator is above threshold
                    new MoveWristToSetpoint(wrist, wristSetpoint),
                    () -> elevator.getCurrentHeight().in(Inches) > 10 || arm.getCurrentAngle().getDegrees() > -40,
                    () -> arm.getCurrentAngle().getDegrees() < 80
                )
            );
    }
    /**
     * Move elevator, arm, and wrist to setpoints in a parallel command group
     */
    public MoveToolingToSetpoint(Elevator elevator, Arm arm, Wrist wrist, ElevatorSetpoint elevatorSetpoint, ArmSetpoint armSetpoint, WristSetpoint wristSetpoint) {
        this(elevator, arm, wrist, elevatorSetpoint, armSetpoint, wristSetpoint, false);
    }


}
