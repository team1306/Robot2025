package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.ArmSetpoints;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoints;
import frc.robot.commands.elevator.MoveElevatorToSetpoint;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.commands.wrist.WristSetpoints;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class PlaceCoral extends ParallelCommandGroup {
    
    /**
     * Places the coral on the reef
     * @param level the level for coral scoring. Must be between 1 and 4
     */
    public PlaceCoral(Elevator elevator, Arm arm, Wrist wrist, Intake intake, IntSupplier level) {
        if(level.getAsInt() < 2) throw new IllegalArgumentException("Level must be between 2 and 4");

        addCommands(
            new MoveElevatorToSetpoint(elevator, ElevatorSetpoints.values()[level.getAsInt() - 1]),
            new SequentialCommandGroup(
                new MoveArmToSetpoint(arm, level.getAsInt() == 4 ? ArmSetpoints.HOVER_L4 : ArmSetpoints.HOVER_L2),
                new WaitUntilCommand(()-> elevator.getCurrentHeight().gt(elevator.getTargetHeight().minus(Inches.of(10)))),
                new MoveWristToSetpoint(wrist, WristSetpoints.VERTICAL)
            ),
            new WaitUntilCommand(()-> elevator.getCurrentHeight().gt(elevator.getTargetHeight().minus(Inches.of(2))))
            .andThen(
                new ParallelCommandGroup(
                    new MoveArmToSetpoint(arm, ArmSetpoints.values()[level.getAsInt()])
            ))
            
        );

    }
}
