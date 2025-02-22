package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autos.DropCoral;
import frc.robot.commands.autos.PlaceCoral;
import frc.robot.commands.wrist.WristSetpoints;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;

public class ScoreL4 extends SequentialCommandGroup{
    public ScoreL4(SwerveSubsystem swerveDrive, Elevator elevator, Wrist wrist, Arm arm){
        addCommands(
            new PlaceCoral(elevator, arm, wrist, 4, WristSetpoints.VERTICAL_L).raceWith(new WaitCommand(5)),
            new DropCoral(elevator, arm, wrist, 4, WristSetpoints.VERTICAL_R),
            new RepeatCommand(
                new InstantCommand(() -> swerveDrive.drive(new ChassisSpeeds(-3, 0, 0))))
                .raceWith(new WaitCommand(1))
        );
    }
}
