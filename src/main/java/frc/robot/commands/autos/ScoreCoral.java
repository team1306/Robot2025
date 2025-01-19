package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;

public class ScoreCoral extends ParallelCommandGroup {
    
    public ScoreCoral(SwerveSubsystem swerve, Elevator elevator, Arm arm, Wrist wrist, FieldLocation reefLocation, int level) {
        
        addCommands(
            swerve.driveToPose(reefLocation.getPose()),
            new PlaceCoral(elevator, arm, wrist, level)
        );
        
    }
}
