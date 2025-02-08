package frc.robot.commands.autos;

import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.SpitCoral;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;

public class ScoreCoral extends SequentialCommandGroup {
    
    /**
     * Places the coral and drives back to score it
     * @param level the level for coral scoring
     */
    public ScoreCoral(SwerveSubsystem swerve, Elevator elevator, Arm arm, Wrist wrist, Intake intake, IntSupplier level) {

        addCommands(
            new PlaceCoral(elevator, arm, wrist, intake, level),
            new ParallelCommandGroup(
                new SpitCoral(intake),
                new InstantCommand(() -> swerve.drive(new Translation2d(-2, 0), 0, false)) //drive backwards
            )
        );
    }
}
