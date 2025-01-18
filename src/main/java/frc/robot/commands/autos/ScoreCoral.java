package frc.robot.commands.autos;

import java.lang.reflect.Field;
import java.util.concurrent.ExecutionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;

public class ScoreCoral extends ParallelCommandGroup {
    
    public ScoreCoral(SwerveSubsystem swerve, Elevator elevator, Arm arm, Wrist wrist, FieldLocation reefLocation, int coralLevel) {
        
        Command placeCoral;
        switch (coralLevel) {
            case 1: placeCoral = new PlaceCoralL1(elevator, arm, wrist); break;
            case 2: placeCoral = new PlaceCoralL2(elevator, arm, wrist); break;
            case 3: placeCoral = new PlaceCoralL3(elevator, arm, wrist); break;
            case 4: placeCoral = new PlaceCoralL4(elevator, arm, wrist); break;
            default: throw new RuntimeException("Coral scoring level must be between 1 and 4");
        }

        addCommands(
            swerve.driveToPose(reefLocation.getPose()),
            placeCoral
        );
        
    }
}
