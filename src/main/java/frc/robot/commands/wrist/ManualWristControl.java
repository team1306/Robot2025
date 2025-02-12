package frc.robot.commands.wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class ManualWristControl extends Command {
     
    private final Wrist wrist;
    private final DoubleSupplier doubleSupplier;
    private final double speed = 0.05;

    /**
    * Manual setting of the wrist angle by a controller
    * @param doubleSupplier controller input from -1 to 1. Negative values lower and positive values raise the setpoint.
    */   
    public ManualWristControl(Wrist wrist, DoubleSupplier doubleSupplier) {
        this.wrist = wrist;
        this.doubleSupplier = doubleSupplier;

        addRequirements(wrist);
    }

    @Override
    public void execute() {
        wrist.setTargetAngle(wrist.getTargetAngle().plus(Rotation2d.fromRadians(doubleSupplier.getAsDouble() * speed)));
    }
}
