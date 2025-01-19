package frc.robot.commands.autos;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
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

public class CollectFromNearestCoralStation extends ParallelCommandGroup {
    
    public CollectFromNearestCoralStation(SwerveSubsystem swerve, Elevator elevator, Arm arm, Wrist wrist) {
        Pose2d nearestCoralStationPosition = swerve.getPose().nearest(Arrays.asList(
            FieldLocation.CORAL_STATION_LEFT, FieldLocation.CORAL_STATION_RIGHT));

        addCommands(
            swerve.driveToPose(nearestCoralStationPosition),
            new MoveElevatorToSetpoint(elevator, ElevatorSetpoint.CORAL_STATION),
            new MoveArmToSetpoint(arm, ArmSetpoint.CORAL_STATION),
            new MoveWristToSetpoint(wrist, WristSetpoint.HORIZONTAL)
        );
    }
}
