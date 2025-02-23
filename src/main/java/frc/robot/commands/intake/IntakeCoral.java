package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;

public class IntakeCoral extends SequentialCommandGroup {
    
    private final double INTAKE_SPEED = 0.7;

    /**
     * Runs the intake until it detects a coral
     */
    public IntakeCoral(Intake intake) {

        addCommands(
            new RunIntake(intake, () -> INTAKE_SPEED).raceWith(new WaitUntilCommand(() -> intake.getSensorReading())),
            new RunIntake(intake, () -> 0.1)
        );
    }
}
