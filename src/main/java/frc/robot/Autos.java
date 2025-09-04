package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmSetpoints;
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

    public Autos(SwerveSubsystem drivebase, Arm arm, Elevator elevator, Intake intake, Wrist wrist) {
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
                .bind("Stow", new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.STOW, ArmSetpoints.STOW, WristSetpoints.HORIZONTAL, true)
                        .raceWith(new WaitCommand(2)))
                .bind("Hover", new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.CORAL_L4, ArmSetpoints.HOVER_L4, WristSetpoints.VERTICAL_L)
                        .raceWith(new WaitCommand(2)))
                .bind("Score", new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.CORAL_L4, ArmSetpoints.CORAL_L4, WristSetpoints.VERTICAL_L)
                        .raceWith(new WaitCommand(2)));
    }

    public AutoRoutine get1CoralL4DriveRoutine(String name) {
        AutoRoutine routine = autoFactory.newRoutine("1 Coral: " + name);
        AutoTrajectory coralPath = routine.trajectory(name);

        RobotContainer.autoRunnable = () -> drivebase.resetOdometry(coralPath.getInitialPose().get());

        routine.active().onTrue(
                Commands.sequence(
                        coralPath.resetOdometry(), coralPath.cmd()
                ));

        return routine;
    }

    public AutoRoutine get2CoralDriveRoutine(String coral1Name, String intermediateName, String coral2Name) {
        AutoRoutine routine = autoFactory.newRoutine("2 Coral: " + coral1Name);
        AutoTrajectory coral1 = routine.trajectory(coral1Name);
        AutoTrajectory intermediateToStation = routine.trajectory(intermediateName);
        AutoTrajectory coral2 = routine.trajectory(coral2Name);

        RobotContainer.autoRunnable = () -> drivebase.resetOdometry(coral1.getInitialPose().get());

        Command intakeCommand = new RunIntake(intake, () -> -1).raceWith(new WaitCommand(0.5));

        routine.active().onTrue(
                Commands.sequence(
                        coral1.resetOdometry(), coral1.cmd()
                )
        );

        coral1.done().onTrue(intermediateToStation.cmd());

        intermediateToStation.atTime("Pickup").onTrue(
                new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.CORAL_STATION, ArmSetpoints.CORAL_STATION, WristSetpoints.HORIZONTAL, false)
                        .alongWith(new RunIntake(intake, () -> -1))
        );

        intermediateToStation.done().onTrue(
                Commands.sequence(
                        intakeCommand, coral2.spawnCmd()
                )
        );

        return routine;
    }

    public AutoRoutine get3CoralDriveRoutine(String path1Name, String path3Name, String path4Name, String path5name) {
        AutoRoutine routine = autoFactory.newRoutine("3 Coral: " + path1Name);
        AutoTrajectory coral1 = routine.trajectory(path1Name);
        AutoTrajectory coral2 = routine.trajectory(path3Name);
        AutoTrajectory intermediateToStation2 = routine.trajectory(path4Name);
        AutoTrajectory coral3 = routine.trajectory(path5name);

        RobotContainer.autoRunnable = () -> drivebase.resetOdometry(coral1.getInitialPose().get());

        Command intakeCommand = new RunIntake(intake, () -> -1).raceWith(new WaitCommand(0.5));

        routine.active().onTrue(
                Commands.sequence(
                        coral1.resetOdometry(), coral1.cmd()
                )
        );

        coral1.atTime("Pickup").onTrue(
                new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.CORAL_STATION, ArmSetpoints.CORAL_STATION, WristSetpoints.HORIZONTAL, false)
                        .alongWith(new RunIntake(intake, () -> -1))
        );

        intermediateToStation2.atTime("Pickup").onTrue(
                new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.CORAL_STATION, ArmSetpoints.CORAL_STATION, WristSetpoints.HORIZONTAL, false)
                        .alongWith(new RunIntake(intake, () -> -1))
        );

        coral2.done().onTrue(
                Commands.sequence(
                        intakeCommand, coral2.spawnCmd()
                )
        );

        intermediateToStation2.done().onTrue(
                Commands.sequence(
                        intakeCommand, coral3.spawnCmd()
                )
        );

        return routine;
    }
}
