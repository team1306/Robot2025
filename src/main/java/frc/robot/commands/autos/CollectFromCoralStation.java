package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ArmSetpoint;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.elevator.MoveElevatorToSetpoint;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.commands.wrist.WristSetpoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;

public class CollectFromCoralStation extends ParallelCommandGroup {
    
    public CollectFromCoralStation(SwerveSubsystem swerve, Elevator elevator, Arm arm, Wrist wrist, boolean left, boolean coralHorazontal) {
        FieldLocation coralStationPosition;
        if (left) coralStationPosition = FieldLocation.CORAL_STATION_LEFT;
        else coralStationPosition = FieldLocation.CORAL_STATION_RIGHT;

        WristSetpoint wristSetpoint;
        if (coralHorazontal) wristSetpoint = WristSetpoint.HORAZONTAL;
        else wristSetpoint = WristSetpoint.VERTICAL;

        addCommands(
            swerve.driveToPose(coralStationPosition.getPose()),
            new MoveElevatorToSetpoint(elevator, ElevatorSetpoint.CORAL_STATION),
            new MoveArmToSetpoint(arm, ArmSetpoint.CORAL_STATION),
            new MoveWristToSetpoint(wrist, wristSetpoint)
        );
    }
}
