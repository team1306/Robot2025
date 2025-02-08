package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ArmSetpoints;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoints;
import frc.robot.commands.elevator.MoveElevatorToSetpoint;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.commands.wrist.WristSetpoints;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;

public class CollectFromNearestCoralStation extends ParallelCommandGroup {
    
    /**
     * Drives to the nearest coral station, brings the intake to the correct position and intakes the coral.
     */
    public CollectFromNearestCoralStation(SwerveSubsystem swerve, Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
        addCommands(
            new DriveToPoseCommand(swerve, () -> swerve.getPose().nearest(FieldLocation.coralStationLocations)),
            new MoveElevatorToSetpoint(elevator, ElevatorSetpoints.CORAL_STATION),
            new MoveArmToSetpoint(arm, ArmSetpoints.CORAL_STATION),
            new MoveWristToSetpoint(wrist, WristSetpoints.HORIZONTAL),
            new RunIntake(intake, () -> 10)
        );
    }
}
