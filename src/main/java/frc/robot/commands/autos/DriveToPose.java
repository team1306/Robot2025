package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Drive to a specific position on the field and then set the last angle to this last angle
 */
public class DriveToPose extends Command{

    private final SwerveSubsystem drivebase;
    private final Supplier<Pose2d> poseSupplier;
    private Command driveCommand;

    public DriveToPose(SwerveSubsystem drivebase, Supplier<Pose2d> poseSupplier){
        this.drivebase = drivebase;
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void initialize(){
        driveCommand = driveToPose(poseSupplier.get());
        driveCommand.schedule();
    }

    @Override
    public void end(boolean interrupted){
        if(interrupted) driveCommand.cancel();
        drivebase.getSwerveController().lastAngleScalar = poseSupplier.get().getRotation().getRadians();
    }

    @Override
    public boolean isFinished(){
        return driveCommand.isFinished();
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
                4, 3,
                drivebase.getSwerveDrive().getMaximumChassisAngularVelocity(), Units.degreesToRadians(1440));
        return driveToPose(pose, constraints);
    }

    public Command driveToPose(Pose2d pose, PathConstraints constraints){
        return AutoBuilder.pathfindToPose(pose, constraints, MetersPerSecond.of(0));
    }
}
