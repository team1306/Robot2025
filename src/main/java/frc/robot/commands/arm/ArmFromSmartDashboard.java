package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.GetValue;

public class ArmFromSmartDashboard extends Command {

    private final Arm arm;
    @GetValue
    private Rotation2d targetRotation;

    /**
     * Sets the target postiton for the arm
     * @param armSetpoint moves the arm to the provided setpoint
     */
    public ArmFromSmartDashboard(Arm arm) {
        this.arm = arm;
        DashboardHelpers.addUpdateClass(this);
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setTargetAngle(targetRotation);
    }
}