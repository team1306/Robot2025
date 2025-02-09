package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;

public class SpitCoral extends ParallelRaceGroup {
    
    private final double INTAKE_SPEED = 0.3;

    /**
     * Reverses the intake until coral is no longer detected.
     */
    public SpitCoral(Intake intake) {
        addCommands(
            new RunIntake(intake, () -> -INTAKE_SPEED),
            new WaitUntilCommand(() -> !intake.getSensorReading())
        );
    }
}
