package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.util.dashboardv3.entry.Config;
import frc.robot.util.dashboardv3.entry.Entry;
import frc.robot.util.dashboardv3.entry.EntryType;
import frc.robot.util.dashboardv3.networktables.mappings.UnitMappings;

public class ArmFromSmartDashboard extends Command {

    private final Arm arm;

    @Entry(type = EntryType.Subscriber)
    @Config(UnitMappings.RotationConfiguration.DEGREES)
    private static Rotation2d targetRotation = Rotation2d.kZero;

    /**
     * Sets the target postiton for the arm
     * @param armSetpoint moves the arm to the provided setpoint
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