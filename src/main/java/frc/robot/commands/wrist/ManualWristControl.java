package frc.robot.commands.wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class ManualWristControl extends Command {
    
    private Wrist wrist;
    private DoubleSupplier doubleSupplier;
    private final double SPEED = 0.01;

    public ManualWristControl(Wrist wrist, DoubleSupplier doubleSupplier) {
        this.wrist = wrist;
        this.doubleSupplier = doubleSupplier;

        addRequirements(wrist);
    }

    @Override
    public void execute() {
        wrist.setTargetAngle(wrist.getTargetAngle().plus(Rotation2d.fromRadians(doubleSupplier.getAsDouble() * SPEED)));
    }
}
