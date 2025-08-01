package frc.robot.commands.arm;

import badgerlog.StructOptions;
import badgerlog.entry.Entry;
import badgerlog.entry.EntryType;
import badgerlog.entry.handlers.StructType;
import badgerlog.entry.handlers.UnitConversion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmFromSmartDashboard extends Command {

    private final Arm arm;

    @Entry(EntryType.Subscriber)
    @UnitConversion("degrees")
    @StructType(StructOptions.MAPPING)
    private static Rotation2d targetRotation = Rotation2d.kZero;

    /**
     * Sets the target postiton for the arm
     */
    public ArmFromSmartDashboard(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setTargetAngle(targetRotation);
    }
}