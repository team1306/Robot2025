package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;

public class ScoreCoral extends SequentialCommandGroup {
    

    public ScoreCoral(SwerveSubsystem swerve, Elevator elevator, Arm arm, Wrist wrist, Intake intake, int level) {

        addCommands(
            new PlaceCoral(elevator, arm, wrist, intake, level),
            new ParallelCommandGroup(
                new ToggleIntake(intake, () -> -1).withTimeout(Seconds.of(1)),
                new InstantCommand(() -> swerve.drive(new Translation2d(-2, 0), 0, false)) //drive backwards
            )
        );
    }
}
