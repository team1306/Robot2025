package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;

public class AutoScoreSequence extends SequentialCommandGroup{
    
    public AutoScoreSequence(SwerveSubsystem swerve, Elevator elevator, Arm arm, Wrist wrist, Intake intake, int level) {


        addCommands(
            new DriveToNearestReef(swerve),
            new PlaceCoral(elevator, arm, wrist, level)
            
        );
    }

}
