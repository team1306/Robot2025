package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToNearestReef extends ParallelCommandGroup {
    
    /**
     * Calculates the nearest reef and drives to it.
     */
    public DriveToNearestReef(SwerveSubsystem swerve) {
        
        addCommands(
            swerve.driveToPose(swerve.getPose().nearest(FieldLocation.reefLocations))
        );
    }

}
