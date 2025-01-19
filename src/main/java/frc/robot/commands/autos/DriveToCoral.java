package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToCoral extends Command {
    
    private final Distance APRIL_TAG_TO_CORAL_STICK_DISTANCE = Inches.of(6.5);

    public DriveToCoral(SwerveSubsystem swerve, CoralLocation coralLocation) {
        //swerve.driveToPose()
    }
}
