package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;

public class IntakeCoral extends ParallelRaceGroup {
    
    private final double INTAKE_SPEED = 0.7;

    /**
     * Runs the intake until it detects a coral
     */
    public IntakeCoral(Intake intake) {

        addCommands(
            new RunIntake(intake, () -> INTAKE_SPEED),
            new WaitUntilCommand(() -> intake.getSensorReading())
        );
    }
}
