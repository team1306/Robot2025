package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

/**
 * Rotate to a specific angle (using the method in {@link frc.robot.commands.autos.DriveToPose})
 */
public class RotateToRotation extends Command {
    
    private final SwerveSubsystem drivebase;
    private final Supplier<Rotation2d> angleSupplier;
    
    public RotateToRotation(SwerveSubsystem drivebase, Supplier<Rotation2d> angleSupplier) {
        this.drivebase = drivebase;
        this.angleSupplier = angleSupplier;
        
        addRequirements(drivebase);
    }
    
    @Override
    public void initialize() {
        drivebase.getSwerveController().lastAngleScalar = angleSupplier.get().getRadians();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
