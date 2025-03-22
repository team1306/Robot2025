package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmSetpoints;
import frc.robot.commands.auto.AutoScoreL1;
import frc.robot.commands.autos.MoveToolingToSetpoint;
import frc.robot.commands.elevator.ElevatorSetpoints;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.wrist.WristSetpoints;
import frc.robot.subsystems.*;

public class Autos {
    private final SwerveSubsystem drivebase;
    private final Arm arm;
    private final Elevator elevator;
    private final Intake intake;
    private final Wrist wrist;
   
    private final AutoFactory autoFactory;
    
    public Autos(SwerveSubsystem drivebase, Arm arm, Elevator elevator, Intake intake, Wrist wrist    ) {
        this.drivebase = drivebase;
        this.arm = arm;
        this.elevator = elevator;
        this.intake = intake;
        this.wrist = wrist;

        autoFactory = new AutoFactory(
                drivebase::getPose, // A function that returns the current robot pose
                drivebase::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
                drivebase::followTrajectory, // The drive subsystem trajectory follower 
                true, // If alliance flipping should be enabled 
                drivebase // The drive subsystem
        )
                .bind("Stow", 
                        new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.STOW, ArmSetpoints.STOW, WristSetpoints.HORIZONTAL, true)
                                .raceWith(new WaitCommand(3)));
    }
    
    public AutoRoutine get1CoralL4DriveRoutine(String name){
        AutoRoutine routine = autoFactory.newRoutine("1 Coral L4: " + name);
        AutoTrajectory coralPath = routine.trajectory(name);
        
        RobotContainer.autoRunnable = () -> drivebase.resetOdometry(coralPath.getInitialPose().get());
        
        coralPath.atTime("Hover").onTrue(
                new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.CORAL_L4, ArmSetpoints.HOVER_L4, WristSetpoints.VERTICAL_L)
        );
        coralPath.atTime("Score").onTrue(
                new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.CORAL_L4, ArmSetpoints.CORAL_L4, WristSetpoints.VERTICAL_L)
                .raceWith(new WaitCommand(3))
        );
        
        routine.active().onTrue(
                Commands.sequence(
                        coralPath.resetOdometry(),
                        coralPath.cmd()
                ));

        return routine;
    }

    public AutoRoutine get1CoralL1DriveRoutine(String name){
        AutoRoutine routine = autoFactory.newRoutine("1 Coral L1: " + name);
        AutoTrajectory coralPath = routine.trajectory(name);

        RobotContainer.autoRunnable = () -> drivebase.resetOdometry(coralPath.getInitialPose().get());
        
        coralPath.atTime("Score").onTrue(new AutoScoreL1(drivebase, elevator, wrist, arm, intake));

        routine.active().onTrue(
                Commands.sequence(
                        coralPath.resetOdometry(),
                        coralPath.cmd()
                ));

        return routine;
    }

    public AutoRoutine get2CoralL4DriveRoutine(String name){
        AutoRoutine routine = autoFactory.newRoutine("2 Coral L4: " + name);
        AutoTrajectory coralPath = routine.trajectory(name);

        RobotContainer.autoRunnable = () -> drivebase.resetOdometry(coralPath.getInitialPose().get());
        
        coralPath.atTime("Hover").onTrue(
                new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.CORAL_L4, ArmSetpoints.HOVER_L4, WristSetpoints.VERTICAL_L)
        );
        coralPath.atTime("Score").onTrue(
                new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.CORAL_L4, ArmSetpoints.CORAL_L4, WristSetpoints.VERTICAL_L, false)
                .raceWith(new WaitCommand(5))
        );

        RunIntake intakeCommand = new RunIntake(intake, () -> -1);
        coralPath.atTime("Intake").onTrue(
                new ParallelCommandGroup(
                        new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.CORAL_STATION, ArmSetpoints.CORAL_STATION, WristSetpoints.HORIZONTAL, false),
                        intakeCommand
                ).raceWith(new WaitCommand(4))
        );

        coralPath.atTime("Wait").onTrue(new WaitCommand(4));

        coralPath.atTime("StopIntake").onTrue(new InstantCommand(() -> intakeCommand.cancel()));

        routine.active().onTrue(
                Commands.sequence(
                        coralPath.resetOdometry(),
                        coralPath.cmd()
                ));

        return routine;
    }
}
