package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.wrist.WristSetpoints;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.autos.PlaceCoral;
public class AutoScore extends SequentialCommandGroup{
    public AutoScore(boolean direction, SwerveSubsystem drivebase, Elevator elevator, Arm arm, Wrist wrist, int level, boolean wristDirection){
        addCommands(
            new AutoAlign(direction, drivebase),

            new PlaceCoral(elevator, arm, wrist, level, wristDirection ? WristSetpoints.VERTICAL_L : WristSetpoints.VERTICAL_R),

            new DropCoral(elevator, arm, wrist, level, wristDirection ? WristSetpoints.VERTICAL_L : WristSetpoints.VERTICAL_R)
        );

    }
}
