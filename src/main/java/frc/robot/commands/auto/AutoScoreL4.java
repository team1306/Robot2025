package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.autos.DropCoral;
import frc.robot.commands.autos.PlaceCoral;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.wrist.WristSetpoints;
import frc.robot.subsystems.*;

public class AutoScoreL4 extends SequentialCommandGroup {
    public AutoScoreL4(SwerveSubsystem swerveDrive, Elevator elevator, Wrist wrist, Arm arm, Intake intake) {
        addCommands(
                new PlaceCoral(elevator, arm, wrist, 4, WristSetpoints.VERTICAL_L)
                        .raceWith(new WaitCommand(3)), new DropCoral(elevator, arm, wrist, 4, WristSetpoints.VERTICAL_L)
                                .raceWith(new WaitCommand(3)), new ParallelRaceGroup(
                                        new RepeatCommand(new InstantCommand(() -> swerveDrive
                                                .drive(new ChassisSpeeds(-3, 0, 0)))), new RunIntake(intake, () -> 1), new WaitCommand(1)
                                )
        );
    }
}
