package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ArmSetpoints;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoints;
import frc.robot.commands.elevator.MoveElevatorToSetpoint;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.commands.wrist.WristSetpoints;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class PlaceCoralL2 extends ParallelCommandGroup {
    
    /**
     * Places coral on L2 or L3. Does not score
     */
    public PlaceCoralL2(Elevator elevator, Arm arm, Wrist wrist, Intake intake, int level) {
        if (level != 2 && level != 3) throw new RuntimeException("Level must be 2 or 3");

        addCommands(
            new MoveElevatorToSetpoint(elevator, level == 2 ? ElevatorSetpoints.CORAL_L2 : ElevatorSetpoints.CORAL_L3),
            
            new MoveArmToSetpoint(arm, ArmSetpoints.HOVER_L2).andThen(
                new StartEventCommand( //move wrist when elevator is above 10 inches
                    new MoveWristToSetpoint(wrist, WristSetpoints.VERTICAL),
                    () -> elevator.getCurrentHeight().gt(Inches.of(10))
                )           
            ),

            new StartEventCommand( //Move arm to setpoint when elevator is within 2 inches of target height
                new MoveArmToSetpoint(arm, ArmSetpoints.CORAL_L2),
                ()-> elevator.getCurrentHeight().gt(elevator.getTargetHeight().minus(Inches.of(2)))
            ) 
        );

    }
}
