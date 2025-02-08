package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.GetValue;

public class WristFromSmartDashboard extends Command {

   
    private final Wrist wrist;
    @GetValue
    private Rotation2d targetRotation = Rotation2d.kZero;
    
    /**
     * Sets the wrist angle
     * @param wristSetpoint setpoint for the wrist to go to
     */
    public WristFromSmartDashboard(Wrist wrist) {
        
        this.wrist = wrist;
        DashboardHelpers.addUpdateClass(this);
        addRequirements(wrist);
    }

    @Override
    public void execute() {
        wrist.setTargetAngle(targetRotation);
    }

   
}