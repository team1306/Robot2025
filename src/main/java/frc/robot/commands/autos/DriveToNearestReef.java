package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToNearestReef extends SequentialCommandGroup {
    
    /**
     * Calculates the nearest reef and drives to it.
     */
    public DriveToNearestReef(SwerveSubsystem swerve) {
        
        addCommands(
            new DriveToPoseCommand(swerve, () -> FieldLocation.getIntermediatePoseFromFinal(swerve.getPose().nearest(FieldLocation.reefLocations))),
            new DriveToPoseCommand(swerve, () -> swerve.getPose().nearest(FieldLocation.reefLocations))
        );
    }

}
