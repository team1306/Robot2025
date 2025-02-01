package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.ArmSetpoint;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.elevator.MoveElevatorToSetpoint;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.commands.wrist.WristSetpoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class PlaceCoral extends ParallelCommandGroup {
    
    /**
     * Places the coral on the reef
     * @param level the level for coral scoring. Must be between 1 and 4
     */
    public PlaceCoral(Elevator elevator, Arm arm, Wrist wrist, Intake intake, int level) {

        if (level < 1 || level > 4) throw new IllegalArgumentException("Coral level must be 1 - 4. (to prevent uneeded code)");


        WristSetpoint wristSetpoint = WristSetpoint.HORIZONTAL;
        if (level > 1) wristSetpoint = WristSetpoint.VERTICAL;
        ElevatorSetpoint elevatorSetpoint = ElevatorSetpoint.values()[level - 1];
        ArmSetpoint armSetpoint = ArmSetpoint.values()[level];
        addCommands(
            new MoveElevatorToSetpoint(elevator, elevatorSetpoint),
            new WaitUntilCommand(()-> elevator.getCurrentHeight().gt(elevator.getTargetHeight().minus(Inches.of(4))))
            .andThen(
                new ParallelCommandGroup(
                    new MoveArmToSetpoint(arm, armSetpoint),
                    new MoveWristToSetpoint(wrist, wristSetpoint)
            ))
            
        );

    }
}
