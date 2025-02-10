package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;


public class ManualArmControl extends Command {
    
    private final Arm arm;
    private final DoubleSupplier doubleSupplier;
    private final double speed = 0.05;
    
    /**
     * Manual setting of the arm angle for controllers
     * @param doubleSupplier controller input from -1 to 1. Negative values lower and positive values raise the setpoint
    */
    public ManualArmControl(Arm arm, DoubleSupplier doubleSupplier) {
        this.arm = arm;
        this.doubleSupplier = doubleSupplier;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setTargetAngle(arm.getTargetAngle().plus(Rotation2d.fromRadians(doubleSupplier.getAsDouble() * speed)));
    }
}
