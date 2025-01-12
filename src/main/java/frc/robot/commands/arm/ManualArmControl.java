package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ManualArmControl extends Command {
    
    private Arm arm;
    private DoubleSupplier doubleSupplier;
    
    public ManualArmControl(Arm arm, DoubleSupplier doubleSupplier) {
        this.arm = arm;
        this.doubleSupplier = doubleSupplier;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        final double SPEED = 0.01; //some random speed factor

        arm.setTargetRotation(arm.getTargetRotation().plus(Rotation2d.fromRadians(doubleSupplier.getAsDouble() * SPEED)));
    }
}
