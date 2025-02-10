package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmSetpoints;
import frc.robot.commands.elevator.ElevatorSetpoints;
import frc.robot.commands.intake.SpitCoral;
import frc.robot.commands.wrist.WristSetpoints;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class ScoreL1 extends ParallelCommandGroup {

    /**
     * Score on L1
     */
    public ScoreL1(Elevator elevator, Intake intake, Arm arm, Wrist wrist){
        addCommands(
            new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.CORAL_L1, ArmSetpoints.CORAL_L1, WristSetpoints.HORIZONTAL),
            new StartEventCommand(new SpitCoral(intake), elevator::atSetpoint, arm::atSetpoint, wrist::atSetpoint)
        );
    }
}
