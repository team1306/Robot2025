package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;

public class Autos {
    private final SwerveSubsystem drivebase;
//    private final Arm arm;
//    private final Elevator elevator;
//    private final Intake intake;
//    private final Wrist wrist;
//    
    private final AutoFactory autoFactory;
    
    public Autos(SwerveSubsystem drivebase) {
        //, Arm arm, Elevator elevator, Intake intake, Wrist wrist
        this.drivebase = drivebase;
//        this.arm = arm;
//        this.elevator = elevator;
//        this.intake = intake;
//        this.wrist = wrist;

        autoFactory = new AutoFactory(
                drivebase::getPose, // A function that returns the current robot pose
                drivebase::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
                drivebase::followTrajectory, // The drive subsystem trajectory follower 
                true, // If alliance flipping should be enabled 
                drivebase // The drive subsystem
        )
                .bind("Score L4", new InstantCommand(() -> System.out.println("AutoL4")))
                .bind("Collect Coral", new InstantCommand(() -> System.out.println("AutoCollect")));
    }
    
    public AutoRoutine getLeaveRoutine() {
        AutoRoutine routine = autoFactory.newRoutine("DriveRoutine");
        AutoTrajectory path = routine.trajectory("Leave Red 2");

        routine.active().onTrue(
                Commands.sequence(
                        path.resetOdometry(),
                        path.cmd()
                ));

        return routine;
    }

    public AutoRoutine getTestDriveRoutine(){
        AutoRoutine routine = autoFactory.newRoutine("DriveRoutine");
        AutoTrajectory pickupTraj = routine.trajectory("TestPath");

        routine.active().onTrue(
                Commands.sequence(
                        pickupTraj.resetOdometry(),
                        pickupTraj.cmd()
                ));

        return routine;
    }
    
    public AutoRoutine get1CoralDriveRoutine(){
        AutoRoutine routine = autoFactory.newRoutine("1 Coral Mid A");
        AutoTrajectory coralPath = routine.trajectory("Middle to A");

        routine.active().onTrue(
                Commands.sequence(
                        coralPath.resetOdometry(),
                        coralPath.cmd()
                ));

        return routine;
    }
}
