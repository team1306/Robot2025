package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmSetpoints;
import frc.robot.commands.autos.MoveToolingToSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoints;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.wrist.WristSetpoints;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;

public class AutoCollectCoral extends SequentialCommandGroup {
    public AutoCollectCoral(SwerveSubsystem swerveDrive, Elevator elevator, Wrist wrist, Arm arm, Intake intake) {
        addCommands(
                new MoveToolingToSetpoint(
                        elevator, arm, wrist, ElevatorSetpoints.CORAL_STATION, ArmSetpoints.CORAL_STATION, WristSetpoints.HORIZONTAL)
                        .raceWith(new WaitCommand(2)), new RunIntake(intake, () -> 1).raceWith(new WaitCommand(2))
        );
    }
}
